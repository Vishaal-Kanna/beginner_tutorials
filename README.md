[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# beginner_tutorials
ROS2 tutorial for a simple publisher subscriber node. Week10 HW is added with service and client. Running nodes using launch file is achieved with parameters.

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

## Use static_turtle_tf2_broadcaster for talk to world
```
colcon build --packages-select learning_tf2_cpp
. install/setup.bash
ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster talk 0 0 1 0 0 0
ros2 run tf2_ros tf2_echo talk world
ros2 run tf2_tools view_frames
```

## To launch rosbag record
```
ros2 launch learning_tf2_cpp rosbag.xml bag_record:=1
cd <your_path_to_repo>/learning_tf2_cpp/Rosbag
ros2 bag info all_topics
ros2 bag play all_topics
```

## To run test on publisher
```
colcon test --event-handlers console_direct+ --packages-select learning_tf2_cpp
```




