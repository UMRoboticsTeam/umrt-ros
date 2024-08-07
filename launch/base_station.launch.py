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
    joy_launch_path = os.path.join(
        get_package_share_directory('ros2_control_demo_example_2'),
        'launch',
        'joy.launch.py',
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path)
    )

    ########################
    # NODE DEFINITIONS
    ########################
    video_decoder_node = launch_ros.actions.Node(
        package='depthai_examples', executable='video_decoder_node',
        output='screen',
        parameters=[{'encoded_video_topic': '/encoded_video'}])

    ########################
    # LAUNCH
    ########################
    base_launch = [
        joy_launch,
        video_decoder_node
    ]

    # Change upon either base or robot launch
    launch_entities = base_launch

    return LaunchDescription(launch_entities)
