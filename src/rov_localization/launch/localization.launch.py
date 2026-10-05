"""EKF (robot_localization) or ground-truth TF, plus static map -> odom.

use_ground_truth:=false  EKF publishes odom -> base_link (default)
use_ground_truth:=true   ground_truth_tf publishes odom -> base_link; the EKF
                         still runs (publish_tf off) so /odometry/filtered can be
                         compared against /odometry/ground_truth.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ekf_cfg = os.path.join(get_package_share_directory('rov_localization'), 'config', 'ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gt = LaunchConfiguration('use_ground_truth')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ground_truth', default_value='false'),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[ekf_cfg, {
                'use_sim_time': use_sim_time,
                'publish_tf': PythonExpression(["'", use_gt, "'.lower() != 'true'"]),
            }]),

        Node(
            package='rov_localization', executable='ground_truth_tf.py', name='ground_truth_tf',
            output='screen', parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_gt)),

        # map -> odom: identity for now; the autonomy stack may take this over.
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
