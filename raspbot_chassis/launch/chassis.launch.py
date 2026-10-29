"""Launch the Raspbot V2 camera view and simple PID line controller."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share = Path(get_package_share_directory('raspbot_chassis'))
    show_view = LaunchConfiguration('show_view').perform(context).lower() == 'true'
    drive = LaunchConfiguration('drive').perform(context).lower() == 'true'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) /
                                           'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + str(share / 'worlds' / 'empty.sdf'),
                          'on_exit_shutdown': 'true'}.items())
    actions = [
        gazebo,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='simulation_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='raspbot_chassis', executable='camera_view', output='screen',
             parameters=[{'show_view': show_view, 'use_sim_time': True}]),
    ]
    if drive:
        tuning = {name: float(LaunchConfiguration(name).perform(context))
                  for name in ('kp', 'ki', 'kd', 'speed', 'max_correction', 'steering_sign')}
        tuning['telemetry_file'] = LaunchConfiguration('telemetry_file').perform(context)
        tuning['use_sim_time'] = True
        actions.append(Node(package='raspbot_chassis', executable='line_controller',
                            output='screen', parameters=[tuning]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('raspbot_chassis'))
    paths = [str(share / 'models'), str(share)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    return LaunchDescription([
        DeclareLaunchArgument('show_view', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('drive', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('kp', default_value='0.04'),
        DeclareLaunchArgument('ki', default_value='0.0'),
        DeclareLaunchArgument('kd', default_value='0.002'),
        DeclareLaunchArgument('speed', default_value='0.02'),
        DeclareLaunchArgument('max_correction', default_value='0.02'),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument('telemetry_file', default_value='~/raspbot_pid.csv'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        OpaqueFunction(function=launch_nodes),
    ])
