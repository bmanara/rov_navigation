# ROV ROS 2 Workspace

Simulation, control, localization, and 3D navigation for the ROV, built on ROS 2 Jazzy and Gazebo Harmonic.

| Package | Purpose |
|---|---|
| `rov_description` | URDF/model of the ROV |
| `rov_gazebo` | Gazebo world, bridges, emulated sensors (DVL, depth, sonar) |
| `rov_control` | Low-level controller (`/cmd_vel`, depth hold) |
| `rov_localization` | EKF (`robot_localization`) and TF |
| `rov_navigation` | Nav2 plugins: 3D planner, controller, goal/progress checkers |
| `rov_bringup` | Top-level launch files and RViz config |

See [src/rov_bringup/README.md](src/rov_bringup/README.md) for the full topic/TF interface.

# Required Packages

- Ubuntu 24.04
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) (installed with `ros-jazzy-ros-gz`)
- ROS packages:
  - `ros-jazzy-ros-gz`
  - `ros-jazzy-robot-localization`
  - `ros-jazzy-navigation2`
  - `ros-jazzy-teleop-twist-keyboard`
  - `ros-jazzy-spatio-temporal-voxel-layer`
- System packages: `libeigen3-dev`, `python3-numpy`, `python3-pytest`, `python3-yaml`

# Installation

```bash
# Dependencies
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-localization ros-jazzy-navigation2 \
  ros-jazzy-teleop-twist-keyboard libeigen3-dev python3-numpy python3-pytest python3-yaml
sudo apt install ros-$ROS_DISTRO-spatio-temporal-voxel-layer # might be best to install from source instead of binary

# Clone and build
mkdir -p ~/ros2_jazzy_ws && cd ~/ros2_jazzy_ws
git clone <repo-url> .
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y   # catches anything missed above
colcon build --symlink-install
source install/setup.bash
```

Quick check:

```bash
ros2 launch rov_bringup sim_control.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # in a second terminal
```

## Troubleshooting

- **Build fails in `ament_package_xml`**: `python3` on your PATH isn't the system one (pyenv/conda). Deactivate it, or build with
  `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.
- **C++ nodes die with `undefined symbol`**: another ROS distro was sourced when you built. Open a fresh terminal, source only `/opt/ros/jazzy/setup.bash`, delete `build/ install/`, and rebuild.
- **Gazebo crashes on startup with Gazebo Jetty also installed**: pass `render_engine:=gz-rendering8-ogre2` to the launch file.

---
# Citations
```
@article{doi:10.1177/1729881420910530,
    author = {Steve Macenski and David Tsai and Max Feinberg},
    title ={Spatio-temporal voxel layer: A view on robot perception for the dynamic world},
    journal = {International Journal of Advanced Robotic Systems},
    volume = {17},
    number = {2},
    year = {2020},
    doi = {10.1177/1729881420910530},
    URL = {https://doi.org/10.1177/1729881420910530}
}
```