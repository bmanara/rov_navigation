# rov_control

Low-level control for the ROV, split into two nodes so each can be tested on its own:

```
/cmd_vel (Twist, body frame)  ──┐
/depth_setpoint (Float64, +down)┼─► velocity_controller ─► /body_wrench ─► thruster_allocator ─► 8 × cmd_thrust [N]
odometry (Odometry)            ──┘     (PIDs, modes)        (WrenchStamped,     (B⁺, saturation)
                                                             base_link)
```

The control math (`Pid`, `ThrusterAllocator`, `VelocityControlLaw`) lives in the
`rov_control_core` library, which doesn't depend on ROS. The nodes are thin wrappers
around it. Unit tests (`test/`) cover the library without Gazebo:

```bash
colcon test --packages-select rov_control && colcon test-result --verbose
```

## velocity_controller

| | Topic | Type | Notes |
|---|---|---|---|
| in | `/cmd_vel` | `geometry_msgs/Twist` | body frame: linear x/y/z, angular z (and optionally angular x/y) |
| in | `/depth_setpoint` | `std_msgs/Float64` | meters, **positive down**. Transient-local QoS. **NaN disables depth hold** |
| in | `odom_topic` param | `nav_msgs/Odometry` | `/odometry/filtered` (default) or `/odometry/ground_truth` |
| out | `/body_wrench` | `geometry_msgs/WrenchStamped` | `base_link` frame |
| out | `~/mode` | `std_msgs/String` | published on every mode change (transient local) |

### Mode-switching rules

**Vertical axis** (`select_vertical_mode()` in `velocity_control_law.cpp`):

| Condition (checked in order) | Mode | What controls z |
|---|---|---|
| `abs(cmd_vel.linear.z) > vz_deadband` | `velocity` | vz PID tracks `cmd_vel.linear.z` |
| else, a finite depth setpoint has been received | `depth_hold` | depth PID → vertical speed ref (≤ `depth.out_limit`) → vz PID |
| else | `velocity` | vz PID holds vz = 0 |

- A nonzero vz command **always wins**, even with a depth setpoint active.
  When vz returns to zero, depth hold resumes and the vehicle goes back to the
  **last setpoint**. The setpoint is not moved to the current depth implicitly.
  To hold a new depth, publish a new setpoint.
- A depth setpoint stays active until a new value or NaN is published.
- On every entry into `depth_hold` the depth PID integrator is reset. The vz
  PID's integrator is kept, since it carries the buoyancy trim.
- Depth is computed as `depth = -odom.z` (odom frame is z-up). This assumes the
  odom origin is at the water surface. In sim the world origin is at the surface
  and the EKF's z comes from the depth sensor, so this holds.

**Roll / pitch**, per axis (`select_attitude_mode()`):

| Condition | Mode |
|---|---|
| `abs(cmd_vel.angular.x/y) > angular_deadband` | `rate`: rate PID tracks the commanded body rate |
| else | `hold_level`: angle PID drives roll/pitch to 0 (D term on measured rate) |

**Yaw:** always yaw-rate control on `cmd_vel.angular.z`. There is no heading hold,
so with `angular.z = 0` the vehicle holds yaw *rate* at zero and heading can
drift slowly. Heading control belongs to whatever sends `/cmd_vel`.

**Timeouts:**
- `/cmd_vel` older than `cmd_vel_timeout` is treated as all-zero. Depth hold
  stays active, so a vehicle with a setpoint holds depth when commands stop.
- Odometry older than `odom_timeout` gives a zero wrench and resets all PIDs.

### Other details

- Velocity loops run on **body-frame** velocities (Odometry twist is in
  `child_frame_id` = `base_link`).
- **Net-buoyancy feed-forward:** `-net_buoyancy` N is applied along world z,
  rotated into the body frame, so the vz integrator only has to absorb model error.
- In depth hold, the world vertical speed reference is converted to body vz with
  `R(2,2)`. This is exact for the z component and ≈1 when level.
- Gains can be changed live: `ros2 param set /velocity_controller vx.kp 50.0`.

## thruster_allocator

Thruster geometry and limits come from `rov_description/config/thrusters.yaml`.
A test in `rov_description` checks that file against the Gazebo model.

- `B` (6×N): column *i* = `[dᵢ; rᵢ × dᵢ]`. Thrusts are `u = B⁺ τ` (minimum-norm
  least squares). The constructor refuses geometry with rank < 6.
- **Saturation:** if any thrust exceeds `max_forward_thrust` / `max_reverse_thrust`,
  the whole thrust vector is scaled down uniformly. The achieved wrench stays
  parallel to the requested one, so the vehicle does less of what was asked
  rather than something different. There is no axis prioritization yet. A
  saturated surge command also scales down the heave/attitude correction.
- `~/achieved_wrench` publishes the wrench after saturation. Compare it with `/body_wrench`.
- **Watchdog:** if no `/body_wrench` arrives for `wrench_timeout`, all
  thrusters are commanded to 0, re-sent at 10 Hz. Gazebo's Thruster system keeps
  the last command it received.

## Tuning

Starting gains in `config/velocity_controller.yaml` were sized from the model's
mass, added mass, and thrust capacity, then checked in sim. To re-tune an axis:

```bash
ros2 launch rov_bringup sim_control.launch.py use_ground_truth:=true gui:=false
ros2 run rov_control step_response.py --axis vx --step 0.3 --csv /tmp/vx.csv
```

`step_response.py` reports rise time, overshoot, 5 % settling time, and
steady-state error for `vx`, `vy`, `vz`, `yaw_rate`, or `depth`.

**Open-loop note:** the model includes added mass (`<fluid_added_mass>`).
Because added mass differs per axis (surge 5.5, sway 12.7, heave 14.6 kg), the
physics produces Munk moments. Open loop, pure surge thrust turns into a slow
spiral. This is real physics and the yaw-rate / attitude loops stabilize it, but
it shows up if you publish raw thrust by hand.
