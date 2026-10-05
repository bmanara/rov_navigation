#!/usr/bin/python3
"""Interface-contract check (CLAUDE.md milestone 7). NOT a navigation stack.

A deliberately dumb proportional "driver": it reads odometry, computes the
error to a target pose, and publishes a saturated 3D body-frame /cmd_vel
(linear x/y/z + angular z) until the vehicle is within tolerance. If this
reaches the target, the control stack honours the interface Nav2 will use.

  ros2 run rov_bringup goto_pose_check.py --x 3 --y -2 --z -2 --yaw 1.57
  (z is odom-frame, z UP: z=-2 means 2 m deep)

Exit code 0 on success, 1 on timeout.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lim):
    return max(-lim, min(lim, v))


class GotoPose(Node):

    def __init__(self, a):
        super().__init__('goto_pose_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, a.odom, self.on_odom, 10)
        self.odom = None
        self.t_start = None
        self.t_in_tol = None
        self.result = None
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        self.odom = msg

    def tick(self):
        if self.odom is None or self.get_clock().now().nanoseconds == 0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now

        p = self.odom.pose.pose.position
        yaw = yaw_of(self.odom.pose.pose.orientation)
        ex, ey, ez = self.a.x - p.x, self.a.y - p.y, self.a.z - p.z
        eyaw = math.atan2(math.sin(self.a.yaw - yaw), math.cos(self.a.yaw - yaw))

        # World-frame horizontal error -> body frame (attitude is held level, so
        # body z ~ world z).
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * ex + s * ey, -s * ex + c * ey

        cmd = Twist()
        cmd.linear.x = clamp(self.a.kp * bx, self.a.vmax)
        cmd.linear.y = clamp(self.a.kp * by, self.a.vmax)
        cmd.linear.z = clamp(self.a.kp * ez, self.a.vmax)
        cmd.angular.z = clamp(self.a.kp_yaw * eyaw, self.a.wmax)

        dist = math.sqrt(ex * ex + ey * ey + ez * ez)
        if dist < self.a.tol and abs(eyaw) < self.a.yaw_tol:
            self.t_in_tol = self.t_in_tol or now
            if now - self.t_in_tol > self.a.hold:
                self.result = True
                cmd = Twist()
        else:
            self.t_in_tol = None
        if now - self.t_start > self.a.timeout:
            self.result = False
            cmd = Twist()
        self.pub.publish(cmd)

        if int(now * 2) != int((now - 0.05) * 2):
            self.get_logger().info(
                f'pos=({p.x:6.2f},{p.y:6.2f},{p.z:6.2f}) yaw={yaw:5.2f}  '
                f'dist={dist:5.2f} m  yaw_err={eyaw:5.2f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, required=True, help='odom frame, z up (negative = depth)')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--odom', default='/odometry/filtered')
    ap.add_argument('--kp', type=float, default=0.5)
    ap.add_argument('--kp-yaw', type=float, default=1.0)
    ap.add_argument('--vmax', type=float, default=0.4)
    ap.add_argument('--wmax', type=float, default=0.5)
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--yaw-tol', type=float, default=0.1)
    ap.add_argument('--hold', type=float, default=2.0, help='seconds within tolerance')
    ap.add_argument('--timeout', type=float, default=90.0)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = GotoPose(a)
    # Wall-clock guard: a stopped/paused sim would otherwise hang a sim-time timeout.
    wall_deadline = time.monotonic() + 3.0 * a.timeout + 10.0
    while rclpy.ok() and node.result is None and time.monotonic() < wall_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.pub.publish(Twist())
    ok = bool(node.result)
    print('PASS: reached target' if ok else 'FAIL: timed out before reaching target')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
