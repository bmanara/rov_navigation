"""Simulation only: Gazebo + ROV + bridge + sensors + localization (+ RViz).

  ros2 launch rov_bringup sim.launch.py [gui:=false] [rviz:=true] [use_ground_truth:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def share(pkg, *p):
        return os.path.join(get_package_share_directory(pkg), *p)

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_ground_truth', default_value='false',
                              description='TF odom->base_link from ground truth instead of EKF'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='gz-rendering8-ogre2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('rov_gazebo', 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'use_sim_time': use_sim_time,
                              'render_engine': LaunchConfiguration('render_engine')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                share('rov_localization', 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_ground_truth': LaunchConfiguration('use_ground_truth')}.items()),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', share('rov_bringup', 'rviz', 'rov.rviz')],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
