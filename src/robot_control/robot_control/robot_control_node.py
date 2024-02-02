#! /usr/src/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy

import json
import serial

import time
import math

import threading
import copy

# Define all the command
EMERGENCY_STOP = {'T': 0}
COORD_CTRL = {"T":2,
              "P1":277.51,"P2":-13.75,"P3":276.5822754,"P4":90,"P5":280,
              "S1":10,"S5":200}
POS_CTRL = {"T":3,"P1":2047,"P2":0,"P3":3500,"P4":1500,"P5":2047,
              "S1":200,"S2":200,"S3":200,"S4":200,"S5":200,
              "A1":60,"A2":60,"A3":60,"A4":60,"A5":60}
GET_DEV_INFO = {"T":4}
GET_ANGTOR_INFO = {"T":5}
ALL_TORQUE_OFF = {"T":8,"P1":0}
ALL_TORQUE_ON = {"T":8,"P1":1}
HELP = {'T': 10}
CORD_CTRL_INIT = {'T': 12}
ST_POS_CTRL_INIT = {'T': 13}

def pythagoras_theorem(depth):
    return depth / math.sqrt(2)

class RobotPosition():
    
    def __init__(self, x, y, z, t, g):
        self.x = x
        self.y = y
        self.z = z
        self.t = t
        self.g = g

    def calculate_new_position(self, callback_data): # assume only 1 object 
        edge = None
        position_2d = None
        for item in callback_data:
            if item['name'] == 'cup' or 'bottle':
                position_2d = item['position']
                distance = item['distance']
                x_distance = position_2d[1]
                y_distance = position_2d[0]
        if position_2d:
            self.x -= (x_distance - 10) # Claw Offset
            self.y += y_distance - 55 # Camera Offset
            self.z -= (distance - 25)
            self.g = 280
            
        self.round()
    def round(self):
        self.x = round(self.x, 2)
        self.y = round(self.y, 2)
        self.z = round(self.z, 2)
        self.t = round(self.t, 2)
        self.g = round(self.g, 2)


    def __str__(self):
        return "x: {}, y: {}, z: {}, t: {}, g: {}".format(self.x, self.y, self.z, self.t, self.g)

    def encode_command(self) -> json:
        GRABBER_SPEED = 200
        STEP_DELAY = 10
        command = {"T":2,"P1":self.x,"P2":self.y,"P3":self.z,"P4":self.t,"P5":self.g,
                   "S1": STEP_DELAY, "S5": GRABBER_SPEED,}
        return json.dumps(command).encode()
    
class RobotControl(Node):
    initial_pos = RobotPosition(213.71, -13.55, 366.29, 75, 180)
    initial_pos_claw_close = RobotPosition(213.71, -13.55, 366.29, 75, 280)
    # initial_pos = RobotPosition(150.91, -25.55, 418.19, 75, 180)
    # initial_pos = RobotPosition(213.71+234-10, -13.55, 366.29-234, 75, 180)
    position = None
    callback_data = None
    def __init__(self):
        super().__init__('robot_control_node')
        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        self.subscription = self.create_subscription(String, '/ObjDetection/Objects', self.listener_callback, 10)
        self.setup_connection()
        self.setup_robot_position()
        self.get_logger().info("Robot Control Node Initialised")
    
    def listener_callback(self, msg):
        self.callback_data = json.loads(msg.data)
        self.get_logger().debug(f"Listener Callback: {self.callback_data}")

    def setup_robot_position(self) -> None:
        self.send_arm_command(self.initial_pos_claw_close.encode_command())
        time.sleep(1)
        self.send_arm_command(self.initial_pos.encode_command())
        self.position = copy.deepcopy(self.initial_pos)

    def setup_connection(self):
        while True:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200)
                break
            except:
                self.get_logger().info("No Serial Connection, Wait for 5s...")
                time.sleep(5)

    def send_arm_command(self, command):
        self.ser.write(command)
        self.get_logger().info("Send Command: {}".format(command))
        response = self.ser.readline()
        self.get_logger().info("Response 1: {}".format(response))
        response = self.ser.readline()
        self.get_logger().info("Response 2: {}\n".format(response))
    
    def calculate_edge_distance(self):
        for item in self.callback_data:
            if item['name'] == 'cup' or 'bottle':
                position_2d = item['position']
                distance = item['distance']
                return pythagoras_theorem(distance)
                

    def start(self):
        while True:
            i = input("Press Y to ACTION, N to QUIT,, R to Reset...")
            self.get_logger().info("Robot Control Node Started")
            # check if callback data is avaliable
            while not self.callback_data:
                time.sleep(0.5)
                self.get_logger().debug("Waiting for callback data...")
            self.get_logger().info(f"Listener Callback: {self.callback_data}")
            self.position.calculate_new_position(self.callback_data)
            self.get_logger().info("Current Position: {}".format(self.initial_pos))
            self.get_logger().info("New Position: {}".format(str(self.position)))

            if (i == 'y'):
                self.send_arm_command(self.position.encode_command())
                time.sleep(2)
                self.setup_robot_position()
            elif (i == 'n'):
                break
            elif (i == 'r'):
                self.setup_robot_position()
            print("\n\n\n\n\n")
                

    


def main():
    rclpy.init()
    robot_control_node = RobotControl()
    robot_control_thread = threading.Thread(target=robot_control_node.start)
    try:
        robot_control_thread.start()
        rclpy.spin(robot_control_node)
    except:
        pass
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


