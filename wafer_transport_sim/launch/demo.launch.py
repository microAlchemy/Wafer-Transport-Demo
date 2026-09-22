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
    layout = LaunchConfiguration('layout').perform(context)
    config = LaunchConfiguration('config').perform(context)
    destination = LaunchConfiguration('destination_station').perform(context)
    four_rooms = layout == 'four_rooms'
    ir_enabled = LaunchConfiguration('ir_enabled').perform(context).lower() == 'true'
    if four_rooms and destination:
        raise RuntimeError('The four-room mission visits ROOM_1 through ROOM_4 in order. '
                           'destination_station is only supported with layout:=single_room.')
    config = config or str(share / 'config' / ('four_rooms.yaml' if four_rooms else 'simulation.yaml'))
    bridge_config = LaunchConfiguration('bridge_config').perform(context)
    bridge_config = bridge_config or str(share / 'config' / ('four_rooms_bridge.yaml' if four_rooms else 'bridge.yaml'))
    gz_args = ['-r ', str(share / 'worlds' / ('four_rooms.sdf' if four_rooms else 'wafer_transport.sdf'))]
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
    executables = (('four_room_controller', 'ir_line_follower') if four_rooms else
                   ('wafer_handler', 'line_follower', 'qr_detector',
                    'docking_controller', 'transport_controller', 'system_manager'))
    for executable in executables:
        if executable == 'ir_line_follower' and not ir_enabled:
            continue
        name = 'transport_controller' if executable == 'four_room_controller' else executable
        overrides = {'use_sim_time': True}
        if name == 'transport_controller' and destination:
            overrides['destination_station'] = destination
        node = Node(package='wafer_transport_sim', executable=executable, name=name,
                    parameters=[config, overrides], output='screen', emulate_tty=True)
        actions.append(node)
        if four_rooms and executable == 'ir_line_follower':
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
        DeclareLaunchArgument('layout', default_value='four_rooms', choices=['four_rooms', 'single_room']),
        DeclareLaunchArgument('ir_enabled', default_value='true', choices=['true', 'false'],
                              description='Disable probes to exercise simulated tape-following recovery'),
        DeclareLaunchArgument('destination_station', default_value='',
                              description='Optional override of destination in config'),
        DeclareLaunchArgument('config', default_value='', description='Defaults to the selected layout configuration'),
        DeclareLaunchArgument('bridge_config', default_value='', description='Defaults to the selected layout bridges'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        OpaqueFunction(function=launch_nodes),
    ])
