# Intelligent Robotic Arm Object Detection and Handling System - Proof of Concept

## Introduction
Welcome to the repository for our proof of concept: an Intelligent Robotic Arm Object Detection and Handling System. This project showcases the integration of advanced computer vision with robotic manipulation, demonstrating the feasibility and potential of automated object handling in various industries.

## Project Description
This proof of concept combines YOLOv8 for object detection with the Waveshark M1 robotic arm for automated handling. Using the Intel RealSense D415 for depth perception and ROS2 for node communication, it represents a significant step forward in robotics and automation.

### Key Features
- Custom object detection with YOLOv8, trained on Roboflow and Google Colab.
- Depth perception and 3D spatial analysis using Intel RealSense D415.
- Seamless communication between camera and robotic arm using ROS2.
- Automated handling with the Waveshark M1 robotic arm.

### Potential Applications
- Smart Farming (Automated Havesting)
- Automated logistics and warehousing
- Precision manufacturing


# Known Issues and Limitations

## Waveshark M1 Robot Arm Accuracy

### Issue
- The Waveshark M1 robot arm currently exhibits a level of inaccuracy that renders it unsuitable for use in a production environment.

### Potential Improvement
- Consideration could be given to upgrading to a 6-axis robot arm equipped with stepper motors, which would likely offer enhanced accuracy and reliability.

## Intel RealSense D415 Depth Camera Limitations

### Issue
- The Intel RealSense D415 camera is not ideally suited for close-range applications, such as those required by the harvesting arm. Its recommended minimum working distance of 40 cm poses a challenge in our setup.

### Workaround and Observation
- Adjusting the RGB resolution to 480 allows for more accurate depth detection at around 30 cm (±1 cm). However, this is still not optimal for our specific requirements.

### Suggested Improvement
- For better performance in close-range detection, the Intel RealSense D400 series could be a more suitable choice. These models are specialized in close-range, accurate, and reliable detection, potentially overcoming the current limitations of the D415.


