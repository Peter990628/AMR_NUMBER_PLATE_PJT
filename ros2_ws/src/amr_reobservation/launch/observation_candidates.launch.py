"""Geometric viewpoints with fixed-frame confirmation; no navigation goals."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('amr_reobservation'))
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_estimator', default_value='true',
                              description='Set false if the side estimator is already running'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/observation_candidates.yaml')),
        DeclareLaunchArgument('estimator_params_file', default_value=str(share / 'config/vehicle_side_estimator.yaml'),
                              description='Side estimator startup YAML, including refinement_limit'),
        DeclareLaunchArgument('stability_params_file', default_value=str(share / 'config/candidate_stability.yaml')),
        Node(package='amr_reobservation', executable='vehicle_side_estimator_node',
             name='vehicle_side_estimator', output='screen',
             parameters=[LaunchConfiguration('estimator_params_file'), sim],
             condition=IfCondition(LaunchConfiguration('start_estimator'))),
        Node(package='amr_reobservation', executable='observation_candidates_node',
             name='observation_candidates', output='screen',
             parameters=[LaunchConfiguration('params_file'), sim,
                         {'output_topic': '/observation_candidates/raw',
                          'marker_topic': '/debug/observation_candidates/raw'}]),
        Node(package='amr_reobservation', executable='observation_candidate_stabilizer_node',
             name='observation_candidate_stabilizer', output='screen',
             parameters=[LaunchConfiguration('stability_params_file'), sim]),
        Node(package='rviz2', executable='rviz2', name='observation_candidates_rviz', output='screen',
             arguments=['-d', str(share / 'rviz/observation_candidates.rviz')],
             parameters=[sim], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
