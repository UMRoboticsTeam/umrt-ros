from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = "ros2_control_demo_example_2"

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
            )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])