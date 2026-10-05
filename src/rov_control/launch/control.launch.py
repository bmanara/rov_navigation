"""velocity_controller + thruster_allocator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_cfg = os.path.join(
        get_package_share_directory('rov_control'), 'config', 'velocity_controller.yaml')
    thruster_cfg = os.path.join(
        get_package_share_directory('rov_description'), 'config', 'thrusters.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ground_truth = LaunchConfiguration('use_ground_truth')

    def controller(odom_topic, condition):
        return Node(
            package='rov_control', executable='velocity_controller',
            name='velocity_controller', output='screen',
            parameters=[control_cfg, {'use_sim_time': use_sim_time,
                                      'odom_topic': odom_topic}],
            condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_ground_truth', default_value='false',
            description='Feed the controller /odometry/ground_truth instead of the EKF'),
        controller('/odometry/ground_truth', IfCondition(use_ground_truth)),
        controller('/odometry/filtered', UnlessCondition(use_ground_truth)),
        Node(
            package='rov_control', executable='thruster_allocator',
            name='thruster_allocator', output='screen',
            parameters=[thruster_cfg, {'use_sim_time': use_sim_time}]),
    ])
