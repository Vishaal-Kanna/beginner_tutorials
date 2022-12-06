from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
                "Parameter_launch_argument", default_value=TextSubstitution(text=str("ENPM808X Publisher")),
                description="Parameter launch default value"
            ),

        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("WARN")),
            description="Logging level"
        ),

        launch_ros.actions.Node(
            package="cpp_srvcli",
            executable="talker_parm",
            name="publisher_parm",
            parameters=[
                {"Parameter_Publisher":  LaunchConfiguration('Parameter_launch_argument')}
            ],
            output="screen",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        
        launch_ros.actions.Node(
            package="cpp_srvcli",
            executable="client",
            name="publisher_parm",
            output="screen",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        launch_ros.actions.Node(
            package="rosbag2",
            executable="client",
            name="publisher_parm",
            output="screen",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )


        
    ])
