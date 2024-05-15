import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'my_bot'

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        remappings=[
            # diff_drive_spawner in ./launch_sim.launch.py creates this topic. -njreichert
            ("/cmd_vel", "diff_controller/cmd_vel_unstamped")
        ],
        # Launch in an xterm session as ros2 launch doesn't support running 
        # nodes that require stdin through a launch file. -njreichert
        prefix="xterm -e",
    )

    # Launch them all!
    return LaunchDescription([
        teleop_node,
    ])
