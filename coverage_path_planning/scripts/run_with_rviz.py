import time
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from coverage_path_planning.action import CoverageAction
from map_gen.srv import GenerateMap
import random

rclpy.init()
node_ = Node("rviz_coverage")
action_client_ = ActionClient(node_, CoverageAction, "coverage_action")
service_client_ = node_.create_client(GenerateMap, "map_gen/generate_map")
line_pub_ = node_.create_publisher(MarkerArray, "coverage_marker", 10)

inter_marker_ = Marker()
inter_marker_.header.frame_id = "map"
inter_marker_.ns = "line"
inter_marker_.id = 0
inter_marker_.type = Marker.LINE_LIST
inter_marker_.action = Marker.ADD
inter_marker_.pose.orientation.w = 1.0
inter_marker_.scale.x = 0.05
inter_marker_.color.r = 1.0
inter_marker_.color.g = 0.0
inter_marker_.color.b = 0.0
inter_marker_.color.a = 1.0

delete_marker_ = Marker()
delete_marker_.id = -1
delete_marker_.ns = "line"
delete_marker_.action = Marker.DELETEALL

num_cycles = 10
min_slope = 15
max_slope = 45
smoothing_it = 3
min_alpha = 0.0
max_alpha = 1.0

random.seed(1234)

for i in range(num_cycles):
    node_.get_logger().info("Processing map {i+1}/{num_cycles}")

    request = GenerateMap.Request()
    request.file_path = "./output.stl"
    request.seed = i
    request.size = 25
    request.max_slope = min_slope + (max_slope - min_slope) * random.random()
    request.smoothing_iterations = smoothing_it
    request.smoothing_alpha = min_alpha + (max_alpha - min_alpha) * random.random()

    service_client_.wait_for_service()
    node_.get_logger().info("Sending MapGen request...")
    future = service_client_.call_async(request)
    rclpy.spin_until_future_complete(node_, future)

    if future.result() is not None:
        node_.get_logger().info(f"Service response: {future.result()}")
    else:
        node_.get_logger().error(f"Service call failed")

    goal = CoverageAction.Goal()
    goal.map_filepath = "./output.stl"

    action_client_.wait_for_server()
    node_.get_logger().info("Sending CoveragePathPlanning request...")
    goal_future = action_client_.send_goal_async(goal)
    rclpy.spin_until_future_complete(node_, goal_future)

    goal_handle = goal_future.result()
    if not goal_handle.accepted:
        node_.get_logger().warn("Goal rejected")
    else:
        node_.get_logger().info("Goal accepted")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node_, result_future)

    result = result_future.result().result
    if result is not None:
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker_)
        line_pub_.publish(marker_array)
        node_.get_logger().info("Action completed successfully")
        marker_array = MarkerArray()
        inter_marker_.points = []
        for p in range(1, len(result.inter_path)-1):
            inter_marker_.points.append(result.inter_path[p])
        marker_array.markers.append(inter_marker_)
        for r in range(len(result.intra_paths)):
            intra_marker_ = Marker()
            intra_marker_.header.frame_id = "map"
            intra_marker_.ns = "line"
            intra_marker_.id = r+1 # start at id 1 because id 0 is inter region
            intra_marker_.type = Marker.LINE_STRIP
            intra_marker_.action = Marker.ADD
            intra_marker_.pose.orientation.w = 1.0
            intra_marker_.scale.x = 0.05
            intra_marker_.color.r = 0.0
            intra_marker_.color.g = 0.0
            intra_marker_.color.b = 1.0
            intra_marker_.color.a = 1.0
            for p in result.intra_paths[r].data:
                intra_marker_.points.append(p)
            marker_array.markers.append(intra_marker_)
        line_pub_.publish(marker_array)
    else:
        node_.get_logger().error("Action execution failed")






