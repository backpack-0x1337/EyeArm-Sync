#! /usr/src/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, HistoryPolicy

import json
import serial

import time

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



class RobotPosition():
    
    def __init__(self, x, y, z, t, g):
        self.x = x
        self.y = y
        self.z = z
        self.t = t
        self.g = g

    def __str__(self):
        return "x: {}, y: {}, z: {}, t: {}, g: {}".format(self.x, self.y, self.z, self.t, self.g)

    def encode_command(self) -> json:
        GRABBER_SPEED = 200
        STEP_DELAY = 10
        command = {"T":2,"P1":self.x,"P2":self.y,"P3":self.z,"P4":self.t,"P5":self.g,
                   "S1": STEP_DELAY, "S5": GRABBER_SPEED,}
        return json.dumps(command).encode()
    
class RobotControl(Node):
    initial_pos = RobotPosition(277.51, -13.75, 276.5822754, 90, 180)
    position = None
    def __init__(self):
        super().__init__('robot_control_node')
        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        # self.subscriptions = self.subscriptions(String, '/ObjDetection/Objects', qos_profile)
        self.setup_connection()
        self.setup_robot_position()
        
        self.get_logger().info("Robot Control Node Initialised")
        
    def setup_robot_position(self) -> None:
        self.send_command_and_get_response(self.initial_pos.encode_command())
        self.position = self.initial_pos

    def setup_connection(self):
        while True:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200)
                break
            except:
                self.get_logger().info("No Serial Connection, Wait for 5s...")
                time.sleep(5)

    def send_command_and_get_response(self, command):
        self.ser.write(command)
        self.get_logger().info("Send Command: {}".format(command))
        response = self.ser.readline()
        self.get_logger().info("Response 1: {}".format(response))
        response = self.ser.readline()
        self.get_logger().info("Response 2: {}\n".format(response))



def main():
    rclpy.init()
    robot_control_node = RobotControl()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
# TODO Set the robot to Inital Position

# TODO Read Object Detection Topic 

# TODO Calculate the x,y,z value require for the robot to move to

# TODO Send the command to the robot

# TODO Read the response from the robot

# TODO Sleep for 10000ms and play a sound
