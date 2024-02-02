#!/usr/bin/env python
# ROS Library
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import String

# Intel RealSense & Yolo Library
from realsense_camera import *
import cv2
from ultralytics import YOLO
from yolo_reader import YoloReader

import numpy as np
import json
import sys

class ObjectDetection(Node):

    def __init__(self, argv, objects:list[str]=["cup", "bottle"]):
        # ROS Initialisation
        super().__init__("Object_Reader")
        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(String, '/ObjDetection/Objects', qos_profile)
        
        # Debug Mode
        if len(argv) > 1 and argv[1] == "DEBUG":
            self.debug = True
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.debug = False

        # Intel RealSense Initialisation
        self.rs = RealsenseCamera()
        self.model = YOLO("yolov8n.pt")
        self.obj_reader = YoloReader(self.model, rs=self.rs)
        self.object_names = objects
        
        
        self.get_logger().info("Object Detector Node Initialised")
        self.start()
        self.cleanup()

    def start(self):
        self.get_logger().info("Object Detector Node Running")
        while True:
                msg = []

                # Read from Camera
                ret, bgr_image, depth_image = self.rs.get_frame_stream()
                if ret is False:
                    continue

                # Get Object Detection Data
                self.obj_reader.predict(bgr_image, depth_image, conf=0.1)

                
                for obj in self.obj_reader.object_list:
                    if obj.name not in self.object_names:
                        continue

                    msg_item = {
                        "name": obj.name,
                        "position": (int(obj.center_point_x), int(obj.center_point_y)),
                        "distance": int(obj.center_point_depth),
                    }
                    msg.append(msg_item)

                    if self.debug: # Draw Box, class info & depth info on the Object
                        cv2.rectangle(bgr_image, (obj.top_left_x, obj.top_left_y), (obj.bottle_right_x, obj.bottom_right_y),
                                    (255, 0, 255), 1)
                        cv2.circle(bgr_image, (obj.center_pixel_x, obj.center_pixel_y), 5, (0, 0, 255), 1)
                        cv2.putText(bgr_image, self.model.names[obj.cls], (obj.top_left_x, obj.top_left_y), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 0, 255), 1)
                        cv2.putText(bgr_image, obj.info_str[0], (obj.top_right_x, obj.top_right_y), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 0, 255), 1)
                        cv2.putText(bgr_image, obj.info_str[1], (obj.center_right_x, obj.center_right_y), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 0, 255), 1)
                        cv2.putText(bgr_image, obj.info_str[2], (obj.bottle_right_x, obj.bottom_right_y), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 0, 255), 1)
                self.send_msg(msg) 
                if self.debug:
                    cv2.imshow("View", bgr_image)
                    if cv2.waitKey(1) == ord('q'):
                        break    
                self.obj_reader.clear_object_list()
                            
        self.get_logger().info("Object Detector Node Stop Running")

    def cleanup(self):
        self.rs.release()
        self.obj_reader.clear_object_list()
        cv2.destroyAllWindows()
        self.get_logger().info("Object Detector Node Cleaned Up")

    def send_msg(self, msg:dict):
        json_data = json.dumps(msg)
        msg = String()
        msg.data = json_data
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published Message {msg}")



def main():
    rclpy.init()
    objReaderNode = ObjectDetection(sys.argv)
    objReaderNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()