"""Stage 2: independently preprocess scans and estimate the nearest valid side candidate."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("amr_reobservation"))
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config/vehicle_side_estimator.yaml"),
            description="Startup parameter YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="amr_reobservation", executable="vehicle_side_estimator_node",
            name="vehicle_side_estimator", output="screen",
            parameters=[LaunchConfiguration("params_file"), {"use_sim_time": sim_time}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="vehicle_side_rviz",
            arguments=["-d", str(share / "rviz/vehicle_side_estimator.rviz")],
            parameters=[{"use_sim_time": sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ])
