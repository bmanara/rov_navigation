from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_rov_navigation = get_package_share_directory('rov_navigation')

    nav2_params = LaunchConfiguration('nav2_params')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rov_navigation, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav2_params}.items(),
    ) 

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_rov_navigation, 'config', 'nav2_config.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        nav2_launch
    ])
