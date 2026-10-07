#!/usr/bin/python3
"""Emulated forward-looking 3D multibeam sonar, CPU ray cast (no GPU / gpu_lidar).

A grid of beams (num_elevation_beams rows x num_beams columns) covering
horizontal_fov_deg x vertical_fov_deg. Each beam returns its nearest echo,
placed along the beam centre (rov_gazebo.sonar_model). Rays are cast against
the static collision primitives of the world SDF (rov_gazebo.raycast), from the
vehicle's ground-truth pose.

Limitations (it's geometry, not acoustics): no multipath, no surface
reverberation, no intensity, and only box/cylinder/sphere collisions in the
world file are visible. Spawned or moving models are not seen.

Input:   /odometry/ground_truth   nav_msgs/Odometry (odom == world)
Output:  /sensors/sonar/points    sensor_msgs/PointCloud2 (x, y, z float32), frame sonar_link
           organized_cloud=false: hits only, height 1, is_dense true
           organized_cloud=true:  height = elevation rows (top first),
                                  width = azimuth columns (right to left), NaN = no return
         /sensors/sonar           sensor_msgs/LaserScan, frame sonar_link: nearest
                                  return per azimuth column (the 3D grid collapsed).
                                  REP-117: +inf = no return, -inf = closer than range_min.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from rov_gazebo.raycast import load_world_primitives, quaternion_to_matrix, rpy_to_matrix
from rov_gazebo.sonar_model import SonarConfig, SonarModel

XYZ_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate('xyz')]


def make_cloud(header, xyz, height, width, dense):
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = height, width
    msg.fields = XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * width
    msg.is_dense = dense
    msg.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
    return msg


class SonarEmulator(Node):

    def __init__(self):
        super().__init__('sonar_emulator')
        p = self.declare_parameter
        world_file = p('world_file', '').value
        exclude = p('exclude_models', ['rov']).value
        self.frame = p('frame_id', 'sonar_link').value
        rate = p('rate', 10.0).value
        self.organized = p('organized_cloud', False).value
        d = SonarConfig()
        cfg = SonarConfig(
            horizontal_fov_deg=p('horizontal_fov_deg', d.horizontal_fov_deg).value,
            num_beams=p('num_beams', d.num_beams).value,
            vertical_fov_deg=p('vertical_fov_deg', d.vertical_fov_deg).value,
            num_elevation_beams=p('num_elevation_beams', d.num_elevation_beams).value,
            sub_rays_azimuth=p('sub_rays_azimuth', d.sub_rays_azimuth).value,
            sub_rays_elevation=p('sub_rays_elevation', d.sub_rays_elevation).value,
            range_min=p('range_min', d.range_min).value,
            range_max=p('range_max', d.range_max).value,
            range_stddev=p('range_stddev', d.range_stddev).value)
        # sonar_link pose in base_link (must match urdf/rov.urdf)
        self.t_bs = np.array(p('mount_xyz', [0.21, 0.0, -0.06]).value, dtype=float)
        self.R_bs = rpy_to_matrix(*p('mount_rpy', [0.0, 0.0, 0.0]).value)

        if not world_file:
            raise RuntimeError('world_file parameter is required')
        self.prims = load_world_primitives(world_file, exclude_models=exclude)
        self.model = SonarModel(cfg)
        self.cfg = cfg
        self.get_logger().info(
            f'3D sonar: {len(self.prims)} collision primitives from {world_file}; '
            f'{cfg.num_elevation_beams}x{cfg.num_beams} beams over '
            f'{cfg.vertical_fov_deg:.0f}x{cfg.horizontal_fov_deg:.0f} deg, '
            f'{self.model.num_rays} rays/scan, {cfg.range_max:.1f} m')

        self.cloud_pub = self.create_publisher(PointCloud2, '/sensors/sonar/points', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/sensors/sonar', 10)
        self.latest = None
        self.create_subscription(Odometry, '/odometry/ground_truth', self.on_odom, 10)
        self.scan_time = 1.0 / rate
        self.create_timer(self.scan_time, self.tick)

    def on_odom(self, msg):
        self.latest = msg

    def tick(self):
        if self.latest is None:
            return
        pose = self.latest.pose.pose
        q = pose.orientation
        R_wb = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        t_wb = np.array([pose.position.x, pose.position.y, pose.position.z])
        scan = self.model.scan(self.prims, t_wb + R_wb @ self.t_bs, R_wb @ self.R_bs)

        header = self.latest.header
        header.frame_id = self.frame
        if self.organized:
            n_el, n_az = self.model.shape
            cloud = make_cloud(header, scan.points.reshape(-1, 3), n_el, n_az, dense=False)
        else:
            pts = scan.valid_points()
            cloud = make_cloud(header, pts, 1, len(pts), dense=True)
        self.cloud_pub.publish(cloud)

        ls = LaserScan()
        ls.header = header
        ls.angle_min = -self.model.h_fov / 2.0
        ls.angle_max = self.model.h_fov / 2.0
        ls.angle_increment = self.model.h_fov / max(self.cfg.num_beams - 1, 1)
        ls.scan_time = self.scan_time
        ls.range_min = float(self.cfg.range_min)
        ls.range_max = float(self.cfg.range_max)
        ls.ranges = scan.scan_2d().astype(np.float32).tolist()
        self.scan_pub.publish(ls)


def main():
    rclpy.init()
    rclpy.spin(SonarEmulator())


if __name__ == '__main__':
    main()
