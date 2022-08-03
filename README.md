# Visual_Operator_for_Turtlesim
Visual Operator for Turtlesim

Robot Programming Midterm Project

# Setup
Environment : Ubuntu 20.04 (VirtualBox) / Ros noetic

Install

1. OpenCV : $ sudo apt install ros-noetic-opencv*

2. usb_cam : $ sudo apt-get install ros-noetic-usb-cam

# Step

Mission #1

- roscore
- rosrun usb_cam usb_cam_node
- rosrun visual_operator my_subscriber

Mission #2

- roscore
- rosrun visual_operator optical_flow

Mission #3

- roscore
- export TURTLEBOT3_MODEL = waffle_pi
- roslaunch turtlebot3_gazebo turtlebot3_world.launch
- rosrun visual_operator my_publisher

# Results
![image](https://user-images.githubusercontent.com/83438707/181257617-e95cb679-73aa-445e-bc98-beb1097b5a93.png)
![image](https://user-images.githubusercontent.com/83438707/181257854-b584b534-b146-46ea-8e0a-b844ba923983.png)
