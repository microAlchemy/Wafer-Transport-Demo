"""Launch the Raspbot V2 chassis in an empty Gazebo world."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('raspbot_chassis'))
    paths = [str(share / 'models'), str(share)]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) /
                                           'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + str(share / 'worlds' / 'empty.sdf'),
                          'on_exit_shutdown': 'true'}.items())
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(paths)),
        gazebo,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='simulation_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'),
                          'use_sim_time': True}]),
    ])
