# Eyes

1. Overview

This is a ROS package that contains the necessary code for using the Eye Robot. With this package, the robot is able to track any face within the camera's field of vision and center that face on the frame. The robot is also able to estimate the depth of the face using stereo vision.



&nbsp;




2. Launch Files

There are 2 ROS launch files locates within the launch folder of the repository

depth.launch - This launch file runs the necessary nodes to get an estimation of the depth (in metres) of any face in the frame. When launched, a frame will pop up where all the faces in the image will be labeled with a green box and the estimated depth of that face in metres. The stereo vision parameters can be edited within this launch file.

face_tracking.launch - This launch file runs the necessary nodes to track any face within the frame. When launched, the robot will start moving the eyes to center the first face it detects within the frame, again labeling it with a green box


&nbsp;


3. Additional Features

The calibration data folder contains the necessary calibration data required for stereo rectification. The weights folder contains the weights for the yolov5 face detection model used to detect faces. 


&nbsp;

4. Nodes

left_display - publishes the image feed and calibration data for the left camera

right_display - publishes the image feed and calibration data for the right camera

face_detector - subscribes to left image feed, calibration data, and the face depth and detects faces labeling them with a green box and the depth estimation 

depth_node - subcribes to the pointcloud publication from the ros stereo_image_proc node to publish the face depth in metres

eye_mover - moves the robot based on the x and y control effort published by the ros pid package


&nbsp;

5. Additional scripts

center.py - a python executable file that sets all the servo positions to 512 to center the eyes

robot.py - contains a robot class to organize all the hardware writes


