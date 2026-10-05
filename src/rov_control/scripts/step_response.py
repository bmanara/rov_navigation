#!/usr/bin/python3
"""Step-response test for one control axis (CLAUDE.md milestone 6).

Holds zero command for --pre seconds, applies a step, records the response, and
prints rise time / overshoot / settling time / steady-state error.

  ros2 run rov_control step_response.py --axis vx --step 0.3
  ros2 run rov_control step_response.py --axis vz --step -0.2
  ros2 run rov_control step_response.py --axis yaw_rate --step 0.5
  ros2 run rov_control step_response.py --axis depth --step 1.5     # setpoint [m, +down]
  ... --csv /tmp/vx.csv    to dump the raw trace

Needs sim_control.launch.py running. Uses sim time.
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64

AXES = ('vx', 'vy', 'vz', 'yaw_rate', 'depth')


def measure(axis, odom):
    t = odom.twist.twist
    return {'vx': t.linear.x, 'vy': t.linear.y, 'vz': t.linear.z,
            'yaw_rate': t.angular.z, 'depth': -odom.pose.pose.position.z}[axis]


def analyse(ts, ys, t_step, y0, target):
    span = target - y0
    after = [(t - t_step, y) for t, y in zip(ts, ys) if t >= t_step]
    if not after or abs(span) < 1e-9:
        return {}
    norm = [(t, (y - y0) / span) for t, y in after]
    t10 = next((t for t, n in norm if n >= 0.1), math.nan)
    t90 = next((t for t, n in norm if n >= 0.9), math.nan)
    peak = max(n for _, n in norm)
    band = 0.05
    settle = math.nan
    for i in range(len(norm)):
        if all(abs(n - 1.0) <= band for _, n in norm[i:]):
            settle = norm[i][0]
            break
    tail = [y for t, y in after[int(len(after) * 0.8):]]
    ss = sum(tail) / len(tail)
    return {'rise_time_10_90': t90 - t10, 'overshoot_pct': max(0.0, (peak - 1.0) * 100.0),
            'settling_time_5pct': settle, 'final_value': ss, 'steady_state_error': target - ss}


class StepTest(Node):

    def __init__(self, a):
        super().__init__('step_response', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_pub = self.create_publisher(
            Float64, '/depth_setpoint',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, a.odom, self.on_odom, 20)
        self.ts, self.ys = [], []
        self.t0 = None
        self.y0 = None
        self.depth_sent = False
        self.done = False
        self.create_timer(0.05, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        if self.t0 is None:
            return
        self.ts.append(self.now_s() - self.t0)
        self.ys.append(measure(self.a.axis, msg))

    def tick(self):
        if self.get_clock().now().nanoseconds == 0:
            return  # waiting for /clock
        if self.t0 is None:
            self.t0 = self.now_s()
        t = self.now_s() - self.t0
        stepping = t >= self.a.pre

        if self.a.axis == 'depth':
            self.cmd_pub.publish(Twist())  # zero velocity, depth hold does z
            if stepping and not self.depth_sent:
                self.depth_pub.publish(Float64(data=self.a.step))
                self.depth_sent = True
        else:
            cmd = Twist()
            if stepping:
                v = self.a.step
                if self.a.axis == 'vx':
                    cmd.linear.x = v
                elif self.a.axis == 'vy':
                    cmd.linear.y = v
                elif self.a.axis == 'vz':
                    cmd.linear.z = v
                elif self.a.axis == 'yaw_rate':
                    cmd.angular.z = v
            self.cmd_pub.publish(cmd)

        if t >= self.a.pre + self.a.duration:
            self.cmd_pub.publish(Twist())
            self.done = True


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--axis', choices=AXES, required=True)
    p.add_argument('--step', type=float, required=True,
                   help='step size (m/s, rad/s) or depth setpoint (m, positive down)')
    p.add_argument('--pre', type=float, default=2.0, help='seconds of zero command first')
    p.add_argument('--duration', type=float, default=10.0, help='seconds after the step')
    p.add_argument('--odom', default='/odometry/ground_truth')
    p.add_argument('--csv', default=None)
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = StepTest(a)
    # Wall-clock guard: a stopped/paused sim would otherwise hang forever.
    wall_deadline = time.monotonic() + 3.0 * (a.pre + a.duration) + 10.0
    while rclpy.ok() and not node.done and time.monotonic() < wall_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.done:
        print('WARNING: sim clock stalled; results are partial')

    pre = [y for t, y in zip(node.ts, node.ys) if t < a.pre]
    y0 = sum(pre[-10:]) / max(1, len(pre[-10:])) if pre else 0.0
    target = a.step
    res = analyse(node.ts, node.ys, a.pre, y0, target)
    print(f'\nStep response: axis={a.axis} from {y0:.3f} to {target:.3f} ({a.odom})')
    for k, v in res.items():
        print(f'  {k:22s} {v:8.3f}')
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', a.axis])
            w.writerows(zip(node.ts, node.ys))
        print(f'  trace written to {a.csv}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
