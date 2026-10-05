#!/usr/bin/python3
"""Emulated depth sensor and DVL from Gazebo ground truth + noise, plus IMU
orientation noise/covariance (what a real IMU driver would provide).

Why emulated:
  * depth: Gazebo has no water-pressure sensor; CLAUDE.md specifies
    ground-truth z + noise.
  * DVL: Gazebo's DopplerVelocityLog sensor does exist and ros_gz_bridge maps it
    to marine_acoustic_msgs/Dvl, but robot_localization needs a
    TwistWithCovarianceStamped, the sensor needs GPU ray casting, and its
    bottom-lock behaviour in a small pool is hard to reason about. The
    emulation is cheap and deterministic. Swapping in the real sensor means: add
    the sensor to model.sdf, bridge it, and convert Dvl -> TwistWithCovarianceStamped.

  * IMU: Gazebo's IMU orientation is noiseless and its orientation covariance
    is all zeros, which would make the EKF trust it absolutely.

Inputs:  /odometry/ground_truth (nav_msgs/Odometry; pose in odom==world, twist in base_link)
         /sensors/imu_raw       (sensor_msgs/Imu, bridged from Gazebo)
Outputs:
  /sensors/imu          sensor_msgs/Imu    orientation + small-angle noise, covariance filled
  /sensors/depth        std_msgs/Float64                         depth [m], POSITIVE DOWN
  /sensors/depth/pose   geometry_msgs/PoseWithCovarianceStamped  z = -depth (odom frame,
                                                                 z up), for the EKF
  /sensors/dvl          geometry_msgs/TwistWithCovarianceStamped velocity of dvl_link,
                                                                 expressed in dvl_link
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class SensorEmulator(Node):

    def __init__(self):
        super().__init__('sensor_emulator')
        self.depth_rate = self.declare_parameter('depth_rate', 20.0).value
        self.depth_stddev = self.declare_parameter('depth_stddev', 0.01).value
        self.dvl_rate = self.declare_parameter('dvl_rate', 10.0).value
        self.dvl_stddev = self.declare_parameter('dvl_stddev', 0.01).value
        # dvl_link position in base_link (must match urdf/rov.urdf)
        self.dvl_offset = np.array(
            self.declare_parameter('dvl_offset', [0.0, 0.0, -0.08]).value)
        # Bottom lock: valid only within range of the floor
        self.floor_z = self.declare_parameter('floor_z', -4.0).value
        self.dvl_min_altitude = self.declare_parameter('dvl_min_altitude', 0.05).value
        self.dvl_max_altitude = self.declare_parameter('dvl_max_altitude', 50.0).value
        self.imu_orientation_stddev = self.declare_parameter(
            'imu_orientation_stddev', 0.005).value  # [rad] per axis
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.dvl_frame = self.declare_parameter('dvl_frame', 'dvl_link').value

        self.rng = np.random.default_rng()
        self.latest = None

        self.depth_pub = self.create_publisher(Float64, '/sensors/depth', 10)
        self.depth_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/sensors/depth/pose', 10)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, '/sensors/dvl', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 50)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.on_odom, 10)
        self.create_subscription(Imu, '/sensors/imu_raw', self.on_imu, 50)
        self.create_timer(1.0 / self.depth_rate, self.publish_depth)
        self.create_timer(1.0 / self.dvl_rate, self.publish_dvl)

    def on_odom(self, msg):
        self.latest = msg

    def on_imu(self, msg):
        sd = self.imu_orientation_stddev
        if sd > 0.0:
            # Perturb by a small random rotation (body frame): q' = q * dq
            rx, ry, rz = self.rng.normal(0.0, sd, 3)
            angle = math.sqrt(rx * rx + ry * ry + rz * rz)
            if angle > 1e-12:
                s = math.sin(angle / 2.0) / angle
                dw, dx, dy, dz = math.cos(angle / 2.0), rx * s, ry * s, rz * s
                q = msg.orientation
                w, x, y, z = q.w, q.x, q.y, q.z
                q.w = w * dw - x * dx - y * dy - z * dz
                q.x = w * dx + x * dw + y * dz - z * dy
                q.y = w * dy - x * dz + y * dw + z * dx
                q.z = w * dz + x * dy - y * dx + z * dw
        var = max(sd, 1e-3) ** 2
        msg.orientation_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        self.imu_pub.publish(msg)

    def publish_depth(self):
        if self.latest is None:
            return
        z = self.latest.pose.pose.position.z
        depth = -z + self.rng.normal(0.0, self.depth_stddev)  # positive down

        self.depth_pub.publish(Float64(data=depth))

        p = PoseWithCovarianceStamped()
        p.header.stamp = self.latest.header.stamp
        p.header.frame_id = self.odom_frame
        p.pose.pose.position.z = -depth  # back to z-up for the EKF
        p.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = 1e6           # x, y unobserved
        cov[14] = self.depth_stddev ** 2
        cov[21] = cov[28] = cov[35] = 1e6
        p.pose.covariance = cov
        self.depth_pose_pub.publish(p)

    def publish_dvl(self):
        if self.latest is None:
            return
        altitude = self.latest.pose.pose.position.z - self.floor_z
        if not (self.dvl_min_altitude <= altitude <= self.dvl_max_altitude):
            return  # no bottom lock

        t = self.latest.twist.twist
        v = np.array([t.linear.x, t.linear.y, t.linear.z])
        w = np.array([t.angular.x, t.angular.y, t.angular.z])
        # dvl_link is axis-aligned with base_link, offset only: v_dvl = v + w x r
        v_dvl = v + np.cross(w, self.dvl_offset)
        v_dvl += self.rng.normal(0.0, self.dvl_stddev, 3)

        m = TwistWithCovarianceStamped()
        m.header.stamp = self.latest.header.stamp
        m.header.frame_id = self.dvl_frame
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = v_dvl
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = self.dvl_stddev ** 2
        cov[21] = cov[28] = cov[35] = 1e6  # no angular rates from a DVL
        m.twist.covariance = cov
        if all(math.isfinite(x) for x in v_dvl):
            self.dvl_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(SensorEmulator())


if __name__ == '__main__':
    main()
