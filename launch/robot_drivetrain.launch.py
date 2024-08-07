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
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='False',
    )

    ########################
    # EXTERNAL LAUNCH FILES
    ########################
    robot_description_launch_path = os.path.join(
        get_package_share_directory('umrt_robot_description'),
        'launch',
        'robot_description.launch.py',
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch_path),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
        }.items()
    )

    drivetrain_launch_path = os.path.join(
        get_package_share_directory('ros2_control_demo_example_2'),
        'launch',
        'diffbot.launch.py',
    )

    drivetrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drivetrain_launch_path),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
        }.items(),
    )

    ########################
    # NODE DEFINITIONS
    ########################

    ########################
    # LAUNCH
    ########################
    robot_launch = [
        use_mock_hardware_arg,
        robot_description_launch,
        drivetrain_launch,
    ]

    launch_entities = robot_launch

    return LaunchDescription(launch_entities)
