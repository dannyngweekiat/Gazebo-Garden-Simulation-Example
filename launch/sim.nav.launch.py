import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("gazebo_garden_simulation_example")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_path, "launch", "sim.launch.py"),
                ),
                launch_arguments={
                    "rviz_config": os.path.join(pkg_path, "rviz", "nav.rviz"),
                }.items(),
            )
        ]
    )
