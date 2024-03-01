# ME495 Sensing, Navigation and Machine Learning For Robotics
* Aditya Nair
* Winter 2023
# Package List
This repository consists of several ROS packages
- `nuturtle_description` - 3D models of the nuturtle for simulation and visualization.
- `nusim` - Loads the world consisting of the robot, arena and obstacles, and simulates the robot motion, control and sensors, akin to a real robot.
- `turtlelib` - CMake Library with functions for control and SLAM of a differential drive robot
- `nuturtle_control` - Handles the control and sensing related to the wheels of the robot
- `nuslam` - Performs EKF SLAM using the turtlelbot