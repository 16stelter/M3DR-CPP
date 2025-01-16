from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    ld = LaunchDescription()

    config_file_path = os.path.join(
        get_package_share_directory('coverage_bringup'), 'config', 'config.yaml'
    )
    config = load_config(config_file_path)

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={"pointcloud_enabled": "true", "align_depth": "true"}.items(),
    )

    camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            str(config["camera"]["x"]),
            str(config["camera"]["y"]),
            str(config["camera"]["z"]),
            str(config["camera"]["roll"]),
            str(config["camera"]["pitch"]),
            str(config["camera"]["yaw"]),
            "world",
            config["camera"]["frame_id"]],
        output="screen",
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info')
            ],
            parameters=[
                {'use_sim_time': False},
                {'config_file': os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml')}
            ],
        output="screen",
    )

    ld.add_action(camera)
    ld.add_action(camera_transform)
    ld.add_action(apriltag)
    return ld



def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    
