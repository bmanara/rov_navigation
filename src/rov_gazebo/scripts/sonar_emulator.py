#!/usr/bin/python3
"""Emulated forward-looking multibeam sonar, CPU ray cast (no GPU / gpu_lidar).

The sonar is a horizontal fan of beams. Each beam is sampled by a grid of
rays across its vertical aperture and horizontal beam width; the beam's range
is the nearest hit among them (a sonar returns the first echo anywhere in its beam). Rays are cast
against the static collision primitives of the world SDF (rov_gazebo.raycast),
from the vehicle's ground-truth pose.

Limitations (it's geometry, not acoustics): no multipath, no surface/floor
reverberation unless the floor is inside the beam, no intensity model, and
only box/cylinder/sphere collisions in the world file are visible. Moving
models are not seen.

Input:   /odometry/ground_truth  nav_msgs/Odometry (odom == world)
Output:  /sensors/sonar          sensor_msgs/LaserScan, frame sonar_link
         REP-117: +inf = no return within range_max, -inf = closer than range_min.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rov_gazebo.raycast import (cast, fan_directions, load_world_primitives,
                                quaternion_to_matrix, rpy_to_matrix)


class SonarEmulator(Node):

    def __init__(self):
        super().__init__('sonar_emulator')
        p = self.declare_parameter
        world_file = p('world_file', '').value
        exclude = p('exclude_models', ['rov']).value
        self.frame = p('frame_id', 'sonar_link').value
        rate = p('rate', 10.0).value
        self.h_fov = math.radians(p('horizontal_fov_deg', 130.0).value)
        self.n_beams = p('num_beams', 64).value
        v_ap = math.radians(p('vertical_aperture_deg', 20.0).value)
        n_el = p('num_elevation_rays', 41).value
        n_az = p('num_azimuth_rays', 3).value
        self.range_min = p('range_min', 0.1).value
        self.range_max = p('range_max', 10.0).value
        self.noise = p('range_stddev', 0.02).value
        # sonar_link pose in base_link (must match urdf/rov.urdf)
        xyz = p('mount_xyz', [0.21, 0.0, -0.06]).value
        rpy = p('mount_rpy', [0.0, 0.0, 0.0]).value

        if not world_file:
            raise RuntimeError('world_file parameter is required')
        self.prims = load_world_primitives(world_file, exclude_models=exclude)
        self.get_logger().info(
            f'Sonar: {len(self.prims)} collision primitives from {world_file}; '
            f'{self.n_beams} beams x {n_az}x{n_el} rays, '
            f'{math.degrees(self.h_fov):.0f} deg fan, {self.range_max:.1f} m')

        self.t_bs = np.array(xyz, dtype=float)
        self.R_bs = rpy_to_matrix(*rpy)
        self.rays_per_beam = n_az * n_el
        self.dirs_sensor = fan_directions(self.h_fov, self.n_beams, v_ap, n_el, n_az)
        self.rng = np.random.default_rng()
        self.latest = None

        self.pub = self.create_publisher(LaserScan, '/sensors/sonar', 10)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.on_odom, 10)
        self.create_timer(1.0 / rate, self.scan)
        self.scan_time = 1.0 / rate

    def on_odom(self, msg):
        self.latest = msg

    def scan(self):
        if self.latest is None:
            return
        pose = self.latest.pose.pose
        R_wb = quaternion_to_matrix(pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w)
        t_wb = np.array([pose.position.x, pose.position.y, pose.position.z])

        R_ws = R_wb @ self.R_bs
        origin = t_wb + R_wb @ self.t_bs
        dirs = self.dirs_sensor @ R_ws.T
        dist = cast(self.prims, np.broadcast_to(origin, dirs.shape), dirs)

        # Beam range = nearest echo across its elevation rays
        ranges = dist.reshape(self.n_beams, self.rays_per_beam).min(axis=1)
        hit = np.isfinite(ranges)
        ranges[hit] += self.rng.normal(0.0, self.noise, int(hit.sum()))
        ranges[ranges > self.range_max] = np.inf
        ranges[hit & (ranges < self.range_min)] = -np.inf

        msg = LaserScan()
        msg.header.stamp = self.latest.header.stamp
        msg.header.frame_id = self.frame
        msg.angle_min = -self.h_fov / 2.0
        msg.angle_max = self.h_fov / 2.0
        msg.angle_increment = self.h_fov / max(self.n_beams - 1, 1)
        msg.time_increment = 0.0
        msg.scan_time = self.scan_time
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SonarEmulator())


if __name__ == '__main__':
    main()
