from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    ld = LaunchDescription()

    robot_prefix = DeclareLaunchArgument(
        "robot_prefix",
        default_value="",
        description="Prefix for the robot namespace",
    )

    coverage_node = Node(
        package="coverage_path_planning",
        executable="CoveragePathPlanning",
        name="coverage_path_planning",
        namespace=LaunchConfiguration("robot_prefix"),
        output="screen",
    )

    navigator = Node(
        package="spline_navigator",
        executable="navigator.py",
        name="navigator",
        namespace=LaunchConfiguration("robot_prefix"),
        output="screen",
    )

    ld.add_action(robot_prefix)
    ld.add_action(coverage_node)
    ld.add_action(navigator)
    
    return ld