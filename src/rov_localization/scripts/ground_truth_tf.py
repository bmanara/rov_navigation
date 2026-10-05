#!/usr/bin/python3
"""Broadcast TF odom -> base_link from /odometry/ground_truth.

Used when use_ground_truth:=true, in place of the EKF's TF.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthTf(Node):

    def __init__(self):
        super().__init__('ground_truth_tf')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.on_odom, 10)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GroundTruthTf())


if __name__ == '__main__':
    main()
