# WATonomous ROV: simulation + low-level control

Gazebo Harmonic simulation and control stack for the MATE ROV. It exposes the
interface an autonomy stack (Nav2, later) drives. Nav2 itself is **not** in this repo.

## Install

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard
cd ~/ros2_jazzy_ws
colcon build --symlink-install
source install/setup.bash
```

> **If `python3` on your PATH is not `/usr/bin/python3`** (pyenv, conda), CMake
> picks the wrong interpreter and the build fails in `ament_package_xml`. Either
> deactivate it in ROS terminals or build with
> `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

## Run

```bash
# sim only (Gazebo GUI, EKF, TF)
ros2 launch rov_bringup sim.launch.py [rviz:=true] [gui:=false]

# sim + control
ros2 launch rov_bringup sim_control.launch.py use_ground_truth:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # i/j/l/, etc.; t = up, b = down; Shift = strafe

# depth hold (meters, positive down); NaN disables it
ros2 topic pub --once /depth_setpoint std_msgs/msg/Float64 "{data: 1.5}" --qos-durability transient_local

# tests
colcon test && colcon test-result --verbose
```

Launch args (all launch files): `gui`, `rviz`, `use_ground_truth`, `use_sim_time`, `render_engine`.

## Interface contract

| Topic / TF | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | body frame; linear x/y/z + angular z (angular x/y → roll/pitch rate, optional) |
| `/depth_setpoint` | `std_msgs/Float64` | meters, **positive down**, transient-local; NaN disables |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF, `odom` → `base_link`, twist in `base_link` |
| `/odometry/ground_truth` | `nav_msgs/Odometry` | Gazebo; `odom` == world, origin at water surface |
| TF `map → odom` | static | identity (replace from the autonomy stack) |
| TF `odom → base_link` | | EKF, or ground truth with `use_ground_truth:=true` |
| `/sensors/imu` | `sensor_msgs/Imu` | `imu_link`, 100 Hz |
| `/sensors/dvl` | `geometry_msgs/TwistWithCovarianceStamped` | `dvl_link`, 10 Hz, emulated |
| `/sensors/depth` | `std_msgs/Float64` | meters, positive down, 20 Hz, emulated |
| `/sensors/depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | same measurement as z-up pose in `odom`, for the EKF |
| `/sensors/sonar/points` | `sensor_msgs/PointCloud2` | `sonar_link`, 10 Hz, 3D multibeam: 64 × 24 beams over 130° × 40°, 0.1–10 m; x/y/z of hits (organized + NaN optional), emulated |
| `/sensors/sonar` | `sensor_msgs/LaserScan` | `sonar_link`, same sonar collapsed to 2D: nearest return per azimuth column |
| `/sensors/camera/image_raw`, `/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `camera_optical_frame`, 15 Hz |

Internal (control):

| Topic | Type | Notes |
|---|---|---|
| `/body_wrench` | `geometry_msgs/WrenchStamped` | `base_link`, velocity_controller → thruster_allocator |
| `/model/rov/joint/thrusterN_joint/cmd_thrust` | `std_msgs/Float64` | N, bridged to Gazebo |
| `/thruster_allocator/thrusts`, `/achieved_wrench` | | debugging |
| `/velocity_controller/mode` | `std_msgs/String` | current vertical / attitude modes |

Control modes and tuning are documented in [rov_control/README.md](../rov_control/README.md).

## Pool world

Pool interior: x ∈ [-10, 10] m, y ∈ [-6, 6] m. Water surface at z = 0, floor at
z = -4. The ROV spawns at (0, 0, -0.5) and floats up if left alone.

| Obstacle | Where | Depth |
|---|---|---|
| pillar (r 0.3 m) | (5, 3) | floor to surface |
| reef_block 2×2×1.5 m | (-4, -3) | on the floor, top at z = -2.5 |
| gate, 2 m wide | (8, -2) | crossbar at z = -1.0; pass underneath |
| shelf 2×2 m plate | (-6, 4) | z = -2.0; pass over or under |
| buoy (r 0.4 m) | (2, -5) | centred at z = -1.5 |

## Packages

| Package | Contents |
|---|---|
| `rov_description` | `models/rov/model.sdf` (Gazebo model + plugins), `urdf/rov.urdf` (TF frames), `config/thrusters.yaml` (geometry, single source of truth); consistency test |
| `rov_gazebo` | `worlds/pool.sdf`, `config/bridge.yaml`, `sensor_emulator.py` (depth, DVL, IMU covariance), `sonar_emulator.py` + `rov_gazebo/{raycast,sonar_model}.py` (3D sonar), `launch/sim.launch.py` |
| `rov_control` | `velocity_controller`, `thruster_allocator`, ROS-free `rov_control_core` lib + gtests, `step_response.py` |
| `rov_localization` | `config/ekf.yaml`, `ground_truth_tf.py`, `launch/localization.launch.py` |
| `rov_bringup` | top-level launches, RViz config, `goto_pose_check.py` (milestone 7 interface check) |

## Simulation notes

- **Buoyancy** uses `graded_buoyancy` (water below z = 0, air above), not
  `uniform_fluid_density`. With a uniform fluid, a positively buoyant vehicle
  would rise forever. Net buoyancy is +1.96 N, and the centre of buoyancy sits
  4 cm above the CoM.
- **Added mass** is set with `<fluid_added_mass>` in the base link's inertial,
  not in the Hydrodynamics plugin. The plugin's explicit added-mass force made
  the sim blow up for this light vehicle.
- **Hydrodynamic coefficients** are placeholders from published BlueROV2 Heavy
  values. Tune them against the real vehicle (milestone 6).
- **Depth and DVL are emulated** from ground truth plus Gaussian noise. See
  `sensor_emulator.py` for the reasoning and how to swap in Gazebo's DVL sensor.
- **Sonar is a CPU ray cast**, not a Gazebo sensor, so it needs no GPU.
  `sonar_emulator.py` loads the world SDF's collision primitives and casts a
  64 × 24 grid of beams (130° × 40°, each sampled by 3 × 3 rays) from the
  ground-truth pose. Each beam reports its nearest echo, placed along the
  beam centre, so lateral error grows with range like a real sonar.
  Limits: it only sees **box / cylinder / sphere collisions in the world
  file** (meshes, spawned or moving models are invisible; unsupported shapes
  log a warning), and it has no acoustics (multipath, surface returns,
  intensity). Parameters: `rov_gazebo/config/sonar.yaml`.
  For a frustum-clearing costmap layer (e.g. STVL), the sensor frustum is
  130° (h) × 40° (v), 0.1–10 m, from `sonar_link`.
- **Multiple Gazebo versions installed** (e.g. Harmonic + Jetty): launch files
  run `gz sim --force-version 8` and default `render_engine:=gz-rendering8-ogre2`.
  Jetty installs an unversioned `libgz-rendering-ogre2.so` in the system lib
  dir, which Harmonic would otherwise load and crash on.
