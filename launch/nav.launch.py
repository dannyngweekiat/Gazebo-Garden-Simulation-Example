import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("gazebo_garden_simulation_example")
    nav2_bringup_path = get_package_share_directory("nav2_bringup")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_path, "launch", "bringup_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": os.path.join(pkg_path, "map", "map.yaml"),
            "params_file": os.path.join(pkg_path, "params", "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([nav])
