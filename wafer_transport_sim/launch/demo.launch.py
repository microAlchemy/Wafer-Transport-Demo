"""Launch the complete self-contained Harmonic demonstration."""
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
    share = Path(get_package_share_directory('wafer_transport_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    config = LaunchConfiguration('config').perform(context)
    destination = LaunchConfiguration('destination_station').perform(context)
    gz_args = ['-r ', str(share / 'worlds/wafer_transport.sdf')]
    if not gui:
        gz_args.insert(0, '-s --headless-rendering ')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) /
                                           'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': ''.join(gz_args), 'on_exit_shutdown': 'true'}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='simulation_bridge', output='screen',
                  parameters=[{'config_file': LaunchConfiguration('bridge_config').perform(context),
                               'use_sim_time': True}])
    actions = [bridge, gazebo]
    for name in ('wafer_handler', 'line_follower', 'qr_detector',
                 'docking_controller', 'transport_controller', 'system_manager'):
        overrides = {'use_sim_time': True}
        if name == 'transport_controller' and destination:
            overrides['destination_station'] = destination
        node = Node(package='wafer_transport_sim', executable=name, name=name,
                    parameters=[config, overrides], output='screen', emulate_tty=True)
        actions.extend([node, RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(
                reason=name + ' exited; stopping the simulation'))]))])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('wafer_transport_sim'))
    paths = [str(share / 'models'), str(share)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('destination_station', default_value='',
                              description='Optional override of destination in config'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/simulation.yaml')),
        DeclareLaunchArgument('bridge_config', default_value=str(share / 'config/bridge.yaml')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        OpaqueFunction(function=launch_nodes),
    ])
