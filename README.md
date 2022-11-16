# beginner_tutorials
ROS2 tutorial for a simple publisher subscriber node

##Assumptions
```
OS: Ubuntu Linux Focal (20.04) 64-bit
ROS2 Distro: Humble Hawksbill
ROS2 Workspace name: ros2_ws
ROS2 Installation Directory: ros2_humble
```

##ROS 2 dependencies
```
ament_cmake
rclcpp
std_msgs
```

##Creating the workspace and building the package
```
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Vishaal-Kanna/beginner_tutorials.git
cd ..
colcon build --packages-select cpp_pubsub
```

##Run Publisher and subsrciber
```
Open a terminal from ros2_ws
 . install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener in a new terminal after sourcing
```





