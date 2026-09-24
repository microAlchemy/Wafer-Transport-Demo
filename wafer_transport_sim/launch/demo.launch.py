"""Launch the complete self-contained Jetty demonstration."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction,
                            RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    distro = os.environ.get('ROS_DISTRO', '')
    if distro and distro != 'lyrical':
        raise RuntimeError(
            f'This project targets ROS 2 Lyrical on Ubuntu 26.04; '
            f'this shell has ROS_DISTRO={distro}. Open a fresh terminal and '
            'source /opt/ros/lyrical/setup.bash and the Lyrical workspace.')
    share = Path(get_package_share_directory('wafer_transport_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    config = LaunchConfiguration('config').perform(context)
    ir_enabled = LaunchConfiguration('ir_enabled').perform(context).lower() == 'true'
    config = config or str(share / 'config' / 'four_rooms.yaml')
    bridge_config = LaunchConfiguration('bridge_config').perform(context)
    bridge_config = bridge_config or str(share / 'config' / 'four_rooms_bridge.yaml')
    gz_args = ['-r ', str(share / 'worlds' / 'four_rooms.sdf')]
    if not gui:
        gz_args.insert(0, '-s --headless-rendering ')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) /
                                           'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': ''.join(gz_args), 'on_exit_shutdown': 'true'}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='simulation_bridge', output='screen',
                  parameters=[{'config_file': bridge_config,
                               'use_sim_time': True}])
    actions = [bridge, gazebo]
    executables = ('four_room_controller', 'ir_line_follower', 'apriltag_detector',
                   'process_cell_controller')
    for executable in executables:
        if executable == 'ir_line_follower' and not ir_enabled:
            continue
        name = 'transport_controller' if executable == 'four_room_controller' else executable
        overrides = {'use_sim_time': True}
        node = Node(package='wafer_transport_sim', executable=executable, name=name,
                    parameters=[config, overrides], output='screen', emulate_tty=True)
        actions.append(node)
        if executable == 'ir_line_follower':
            continue  # Controller can emulate tape following if the IR process exits.
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(
                reason=name + ' exited; stopping the simulation'))])))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('wafer_transport_sim'))
    paths = [str(share / 'models'), str(share)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('ir_enabled', default_value='true', choices=['true', 'false'],
                              description='Disable probes to exercise simulated tape-following recovery'),
        DeclareLaunchArgument('config', default_value='', description='Defaults to the eleven-glovebox configuration'),
        DeclareLaunchArgument('bridge_config', default_value='', description='Defaults to the eleven-glovebox bridges'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        OpaqueFunction(function=launch_nodes),
    ])
