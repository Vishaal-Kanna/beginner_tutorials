# beginner_tutorials
ROS2 tutorial for a simple publisher subscriber node

## Assumptions
```
OS: Ubuntu Linux Focal (20.04) 64-bit
ROS2 Distro: Humble Hawksbill
ROS2 Workspace name: ros2_ws
ROS2 Installation Directory: ros2_humble
```

## ROS 2 dependencies
```
ament_cmake
rclcpp
std_msgs
```

## Creating the workspace and building the package
```
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Vishaal-Kanna/beginner_tutorials.git
cd ..
colcon build --packages-select cpp_pubsub
```

## Run Publisher and subsrciber
```
Open a terminal from ros2_ws
 . install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener in a new terminal after sourcing
```

## Run Server and Client
```
colcon build --packages-select cpp_srvcli
Open a terminal from ros2_ws
 . install/setup.bash
ros2 run cpp_srvcli client
```

To modify the output from publishing the string and use it to add two numbers using the client
```
ros2 service call /add_two_ints_v2 example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## Launch publisher with a parameter
```
ros2 launch cpp_srvcli launch.py Parameter_launch_argument:="Hi there" log_level:="INFO"
```

## Use rqt console with
```
ros2 run rqt_console rqt_console
```






