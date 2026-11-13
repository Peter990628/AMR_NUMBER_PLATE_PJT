"""Stage 3: CPU ONNX plate detection and C++ observation-quality evaluation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('amr_reobservation'))
    return LaunchDescription([
        DeclareLaunchArgument('model_path', description='Absolute path to exported plate ONNX model'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/plate_observation.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='amr_reobservation', executable='plate_observation_node',
             name='plate_observation', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
