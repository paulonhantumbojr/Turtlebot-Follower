# Turtlebot3 Follower Code

This repository provides code for a Turtlebot3 waffle pi follower project. The project aims to have two turtlebot waffle pi versions, one guiding and the other following the guide based on a pattern mounted in it. The members responsible for developing this project are:
- Matthew Spender (@mspenr) 
- Julian Taffa (@V370c1ty)
- Paulo Nhantumbo Junior (@pnhantumbojr)

# Execution
The code is set up in two ways, choose either that suits better depending on the language you are familiarised with.

## MATLAB Execution
To run, the MATLAB Setup makes use of the MATLAB Image Processing and Computer Vision Toolboxes to acquire the features from the environment. Before starting, ensure that the [Image Processing Toolbox](https://au.mathworks.com/products/image.html), [Computer Vision Toolbox](https://au.mathworks.com/products/computer-vision.html), and [ROS Toolbox](https://au.mathworks.com/products/ros.html) are installed. 

## ROS Execution
The ROS setup makes use of Python and OpenCV Software to extract the patterns from an image. Before starting, ensure that the necessary OpenCV packages are installed. These include the [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/noetic) and the [cv_camera](https://github.com/OTL/cv_camera) packages (**Note:** This was done in ROS Noetic. Refer to your specific ROS Distro when installing the aforementioned packages).

# Setup
1. To set up the waffle pi, refer to [`TURTLEBOTconnection.md`](https://github.com/mspenr/S-C-Group-Assessment-Code/blob/main/TURTLEBOTconnection.md).
2. If the set up is successful: <br/>
  - **MATLAB Setup**: Download the respective MATLAB scripts and run `run_turtlebot.m`. 
  - **ROS with Python Setup**: Setup the `turtlebot_follower` package and on a terminal run `rosrun turtlebot_follower_ros feature_follower.py`. <br/> <br/>
 **Note:** Adjust the topics depending on what camera is being used.
 
  Refer to the code explanation in [`Code.md`](https://github.com/mspenr/S-C-Group-Assessment-Code/blob/main/Code.md).
