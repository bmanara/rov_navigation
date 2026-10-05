"""Gazebo Harmonic + ROV + ros_gz bridge + robot_state_publisher + emulated sensors.

Does NOT start control or localization; see rov_bringup for the full stacks.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('rov_gazebo')
    desc_share = get_package_share_directory('rov_description')

    model_sdf = os.path.join(desc_share, 'models', 'rov', 'model.sdf')
    with open(os.path.join(desc_share, 'urdf', 'rov.urdf')) as f:
        robot_description = f.read()

    world = LaunchConfiguration('world')
    render_engine = LaunchConfiguration('render_engine')
    use_sim_time = LaunchConfiguration('use_sim_time')

    resource_path = os.pathsep.join(
        p for p in [os.path.join(desc_share, 'models'),
                    os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p)

    # `--force-version 8` pins Gazebo Harmonic even when a newer Gazebo
    # (e.g. Jetty) is installed alongside it and is the CLI default.
    #
    # Server (-s) and GUI (-g) run as separate processes on purpose. Plain
    # `gz sim` forks the server into its own process group; if launch has to
    # SIGKILL the parent during a slow shutdown, that server is orphaned and
    # keeps running, and the next launch then talks to two `pool` worlds at
    # once (ROV appears in random places). With -s / -g, gz runs in-process,
    # so launch's signals reach the real server and GUI.
    gz_common = ['gz', 'sim', '--force-version', '8', '-v', '3']
    gz_server_cmd = gz_common + ['-s', '-r', '--render-engine-server', render_engine]
    gz_server = ExecuteProcess(
        cmd=gz_server_cmd + [world], output='screen', sigterm_timeout='10',
        condition=IfCondition(LaunchConfiguration('gui')))
    gz_server_headless = ExecuteProcess(
        cmd=gz_server_cmd + ['--headless-rendering', world], output='screen',
        sigterm_timeout='10', condition=UnlessCondition(LaunchConfiguration('gui')))
    gz_gui = ExecuteProcess(
        cmd=gz_common + ['-g', '--render-engine-gui', render_engine],
        output='screen', condition=IfCondition(LaunchConfiguration('gui')))

    def warn_stale_server(context):
        # Orphans from older launches / crashes (e.g. kill -9) still interfere.
        try:
            out = subprocess.run(['pgrep', '-af', '^gz sim'], capture_output=True,
                                 text=True, check=False).stdout.strip()
        except OSError:
            return []
        if not out:
            return []
        return [LogInfo(msg='WARNING: Gazebo is already running; the ROV may spawn into '
                            'the stale world. Stop it with: pkill -f "^gz sim"\n' + out)]

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'pool', '-name', 'rov', '-file', model_sdf,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(gazebo_share, 'config', 'bridge.yaml'),
                     'use_sim_time': use_sim_time}])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])

    sensors = Node(
        package='rov_gazebo', executable='sensor_emulator.py', name='sensor_emulator',
        output='screen', parameters=[{'use_sim_time': use_sim_time}])

    # Emulated forward-looking sonar: CPU ray cast against the world's collision
    # primitives (no gpu_lidar). Reads the same world file Gazebo loads.
    sonar = Node(
        package='rov_gazebo', executable='sonar_emulator.py', name='sonar_emulator',
        output='screen',
        parameters=[os.path.join(gazebo_share, 'config', 'sonar.yaml'),
                    {'world_file': world, 'use_sim_time': use_sim_time}])

    # Closing the GUI window or the server exiting ends the whole launch.
    shutdown_on_gz_exit = [
        RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown())]))
        for gz in (gz_server, gz_server_headless, gz_gui)]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(gazebo_share, 'worlds', 'pool.sdf')),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument(
            'render_engine', default_value='gz-rendering8-ogre2',
            description="Versioned name avoids loading another Gazebo's ogre2 plugin "
                        "when several Gazebo versions are installed; 'ogre2' also works "
                        'on a clean Harmonic install'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=warn_stale_server),
        gz_server, gz_server_headless, gz_gui, *shutdown_on_gz_exit,
        spawn, bridge, rsp, sensors, sonar,
    ])
