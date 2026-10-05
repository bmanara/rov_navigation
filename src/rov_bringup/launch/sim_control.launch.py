"""Simulation + low-level control. Drive it with /cmd_vel (3D) and /depth_setpoint.

  ros2 launch rov_bringup sim_control.launch.py use_ground_truth:=true
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # t / b = up / down
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def share(pkg, *p):
        return os.path.join(get_package_share_directory(pkg), *p)

    args = ['gui', 'rviz', 'use_ground_truth', 'use_sim_time', 'render_engine']
    defaults = {'gui': 'true', 'rviz': 'true', 'use_ground_truth': 'false',
                'use_sim_time': 'true', 'render_engine': 'gz-rendering8-ogre2'}

    return LaunchDescription([
        *[DeclareLaunchArgument(a, default_value=defaults[a]) for a in args],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('rov_bringup', 'launch', 'sim.launch.py')),
            launch_arguments={a: LaunchConfiguration(a) for a in args}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('rov_control', 'launch', 'control.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_ground_truth': LaunchConfiguration('use_ground_truth')}.items()),
    ])
