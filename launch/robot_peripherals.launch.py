import os

from pkg_resources import PkgResourcesDeprecationWarning

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch_ros.actions
import launch_ros.descriptions

from ament_index_python import get_package_share_directory

def generate_launch_description():
    ########################
    # PARAMETERS
    ########################

    ########################
    # EXTERNAL LAUNCH FILES
    ########################
    camera_launch_directory = os.path.join(get_package_share_directory('depthai_examples'),
                                           'launch/mobile_publisher.launch.py')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_directory)
    )

    ########################
    # NODE DEFINITIONS
    ########################
    gps_launch = Node(
        package='gpsx',
        executable='gps_node',
        output='screen',
        parameters=[
            {'comm_port': '/dev/ttyUSB0'},
            {'comm_speed': 4800}
        ]
    )

    ########################
    # LAUNCH
    ########################
    robot_launch = [
        camera_launch,
        gps_launch
    ]

    launch_entities = robot_launch

    return LaunchDescription(launch_entities)
