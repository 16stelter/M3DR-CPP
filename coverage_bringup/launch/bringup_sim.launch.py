from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    gazebo_path = get_package_share_directory("ros2_gazebo_sandbox")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(gazebo_path, "worlds", "empty.sdf"),
        description="Path to the Gazebo world file",
    )

    coverage_node = Node(
        package="coverage_path_planning",
        executable="CoveragePathPlanning",
        name="coverage_path_planning",
        output="screen",
    )

    map_gen_node = Node(
        package="map_gen",
        executable="MapGen",
        name="map_gen",
        output="screen",
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_path, "launch", "single.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    navigator = Node(
        package="navigator",
        executable="navigator.py",
        name="navigator",
        output="screen",
    )

    supervisor_controller = Node(
        package="coverage_bringup",
        executable="supervisor_controller.py",
        name="supervisor_controller",
        output="screen"
    )

    
    ld.add_action(sim_world)
    ld.add_action(coverage_node)
    ld.add_action(map_gen_node)
    ld.add_action(simulator)
    ld.add_action(navigator)
    ld.add_action(supervisor_controller)

    return ld