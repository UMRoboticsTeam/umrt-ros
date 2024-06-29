import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():
    ########################
    # EXTERNAL LAUNCH FILES
    ########################

    ########################
    # NODE DEFINITIONS
    ########################

    ########################
    # LAUNCH
    ########################
    launch_entities = []

    return LaunchDescription(launch_entities)
