#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock
from visualization_msgs.msg import Marker, MarkerArray
from coverage_path_planning.action import CoverageAction
from coverage_path_planning.msg import TrialMetadata
from map_gen.srv import GenerateMap
import random

rclpy.init()
node_ = Node("coverage")
action_client_ = ActionClient(node_, CoverageAction, "coverage_action")
service_client_ = node_.create_client(GenerateMap, "map_gen/generate_map")
metadata_pub_ = node_.create_publisher(TrialMetadata, "trial_metadata", 10)
line_pub_ = node_.create_publisher(MarkerArray, "coverage_marker", 10)
clock_ = Clock()

num_cycles = 1
min_slope = 20
max_slope = 45
smoothing_it = 3
min_alpha = 0.0
max_alpha = 1.0

seed = 12345
random.seed(seed)

start_time = clock_.now()
node_.get_logger().info(f"Processing map")

trial_metadata = TrialMetadata()

request = GenerateMap.Request()
request.file_path = "./output.stl"
request.seed = seed
request.size = 15
request.max_slope = min_slope + (max_slope - min_slope) * random.random()
request.smoothing_iterations = smoothing_it
request.smoothing_alpha = min_alpha + (max_alpha - min_alpha) * random.random()

trial_metadata.id = 0
trial_metadata.size = request.size
trial_metadata.max_slope = request.max_slope
trial_metadata.smoothing_iterations = request.smoothing_iterations
trial_metadata.smoothing_alpha = request.smoothing_alpha

service_client_.wait_for_service()
node_.get_logger().info("Sending MapGen request...")
future = service_client_.call_async(request)
rclpy.spin_until_future_complete(node_, future)

if future.result() is not None:
    node_.get_logger().info(f"Service response: {future.result()}")
else:
    node_.get_logger().error(f"Service call failed")

goal = CoverageAction.Goal()
goal.map_filepath = "./src/ros2_gazebo_sandbox/models/output.stl"
goal.runs = 3

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
    node_.get_logger().info("Action completed successfully")

    trial_metadata.inter_cost = result.inter_cost
    trial_metadata.intra_costs = result.intra_costs
    trial_metadata.total_cost = result.total_cost
    total_nodes = 0
    for i in range(len(result.region_order)):
        total_nodes += len(result.intra_paths[result.region_order[i]].data)
        trial_metadata.num_nodes.append(len(result.intra_paths[i].data))
    trial_metadata.num_regions = len(result.region_order)
    trial_metadata.total_nodes = total_nodes

    end_time = clock_.now()
    duration = Duration()
    duration.sec = int((end_time - start_time).nanoseconds // 1e9)
    duration.nanosec = int((end_time - start_time).nanoseconds % 1e9)
    trial_metadata.duration = duration

    metadata_pub_.publish(trial_metadata)
    node_.get_logger().info(f"Finished trial with  {total_nodes} nodes.")
else:
    node_.get_logger().error("Action execution failed")






