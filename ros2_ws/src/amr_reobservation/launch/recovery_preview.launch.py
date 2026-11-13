"""Mission-context preview markers. Does not launch navigation or move the AMR."""
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
        DeclareLaunchArgument('mock', default_value='true',
                              description='Explicit manual hint/initial approach fixture; disable for upstream input'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/recovery_preview.yaml')),
        DeclareLaunchArgument('mock_params_file', default_value=str(share / 'config/mock_recovery_request.yaml')),
        Node(package='amr_reobservation', executable='recovery_preview_node',
             name='recovery_preview', output='screen', parameters=[LaunchConfiguration('params_file'), sim]),
        Node(package='amr_reobservation', executable='mock_recovery_request.py',
             name='mock_recovery_request', output='screen',
             parameters=[LaunchConfiguration('mock_params_file'), sim],
             condition=IfCondition(LaunchConfiguration('mock'))),
        Node(package='rviz2', executable='rviz2', name='recovery_preview_rviz', output='screen',
             arguments=['-d', str(share / 'rviz/recovery_preview.rviz')],
             parameters=[sim], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
