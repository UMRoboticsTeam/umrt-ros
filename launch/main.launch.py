import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():
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

    ########################
    # LAUNCH
    ########################
    launch_entities = [camera_launch]

    return LaunchDescription(launch_entities)
