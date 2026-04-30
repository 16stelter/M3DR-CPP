#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock
from visualization_msgs.msg import Marker, MarkerArray
from coverage_path_planning.action import CoverageAction, PathAction, FlightAction
from coverage_path_planning.msg import TrialMetadata
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from map_gen.srv import GenerateMap
import random, math

rclpy.init()
node_ = Node("rviz_coverage")
node_.declare_parameter("use_global_tsp", False)
use_global_tsp = node_.get_parameter('use_global_tsp').get_parameter_value().bool_value
action_client_ = ActionClient(node_, CoverageAction, "coverage_action")
ground_action_client_ = ActionClient(node_, PathAction, "/multimodal/follow_path_ground")
air_action_client_ = ActionClient(node_, PathAction, "/multimodal/follow_path_air")
flight_level_action_client_ = ActionClient(node_, FlightAction, "/multimodal/fly_to_goal")
service_client_ = node_.create_client(GenerateMap, "map_gen/generate_map")
metadata_pub_ = node_.create_publisher(TrialMetadata, "trial_metadata", 10)
line_pub_ = node_.create_publisher(MarkerArray, "coverage_marker", 10)
clock_ = Clock()

inter_marker_ = Marker()
inter_marker_.header.frame_id = "odom"
inter_marker_.ns = "line"
inter_marker_.id = 2
inter_marker_.type = Marker.LINE_LIST
inter_marker_.action = Marker.ADD
inter_marker_.pose.orientation.w = 1.0
inter_marker_.scale.x = 0.3
inter_marker_.color.r = 1.0
inter_marker_.color.g = 0.0
inter_marker_.color.b = 0.0
inter_marker_.color.a = 1.0

delete_marker_ = Marker()
delete_marker_.id = -1
delete_marker_.ns = "line"
delete_marker_.action = Marker.DELETEALL

min_slope = 45
max_slope = 45
smoothing_it = 3
min_alpha = 0.3
max_alpha = 0.3
flight_speed = 1.0
flight_rot_speed = 0.9
size = 10
scale = 3.0
mode = 0

seed = 898720953
random.seed(seed)

max_height = -math.inf
start_time = clock_.now()
node_.get_logger().info("Processing map...")

trial_metadata = TrialMetadata()

request = GenerateMap.Request()
request.file_path = "./output.stl"
#request.in_file_path = "/home/sebastian/ros2ws/large_0.stl"
#request.in_file_reduction_factor = 0.005
request.seed = seed
request.size = size
request.scale = scale
request.mode = mode
request.max_slope = min_slope + (max_slope - min_slope) * random.random()
request.smoothing_iterations = smoothing_it
request.smoothing_alpha = min_alpha + (max_alpha - min_alpha) * random.random()

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
goal.map_filepath = "./output.stl"
goal.runs = 3
goal.use_global_tsp = use_global_tsp

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
    make_red = result.make_red
    if len(make_red) > 0:
        trial_metadata.total_nodes = len(result.intra_paths[0].data)
        for p in range(len(result.intra_paths[0].data)-1):
            intra_marker_ = Marker()
            intra_marker_.header.frame_id = "odom"
            intra_marker_.ns = "line"
            intra_marker_.id = p+3 # start at id 3 because id 2 is inter region
            intra_marker_.type = Marker.LINE_STRIP
            intra_marker_.action = Marker.ADD
            intra_marker_.pose.orientation.w = 1.0
            intra_marker_.scale.x = 0.3
            if make_red[p]:
                intra_marker_.color.r = 1.0
                intra_marker_.color.g = 0.0
                intra_marker_.color.b = 0.0
            else:
                intra_marker_.color.r = 0.0
                intra_marker_.color.g = 0.0
                intra_marker_.color.b = 1.0
            intra_marker_.color.a = 1.0
            intra_marker_.points.append(result.intra_paths[0].data[p])
            intra_marker_.points.append(result.intra_paths[0].data[p+1])
            marker_array.markers.append(intra_marker_)
    else:
        for p in range(1, len(result.inter_path)-1):
            inter_marker_.points.append(result.inter_path[p])
        marker_array.markers.append(inter_marker_)
        for r in range(len(result.intra_paths)):
            intra_marker_ = Marker()
            intra_marker_.header.frame_id = "odom"
            intra_marker_.ns = "line"
            intra_marker_.id = r+3 # start at id 3 because id 2 is inter region
            intra_marker_.type = Marker.LINE_STRIP
            intra_marker_.action = Marker.ADD
            intra_marker_.pose.orientation.w = 1.0
            intra_marker_.scale.x = 0.3
            intra_marker_.color.r = 0.0
            intra_marker_.color.g = 0.0
            intra_marker_.color.b = 1.0
            intra_marker_.color.a = 1.0
            for p in result.intra_paths[r].data:
                if p.z > max_height:
                    max_height = p.z
                intra_marker_.points.append(p)
            marker_array.markers.append(intra_marker_)

    # show the stl
    map_marker = Marker()
    map_marker.header.frame_id = "odom"
    map_marker.ns = "map"
    map_marker.id = 0
    map_marker.type = Marker.MESH_RESOURCE
    map_marker.action = Marker.ADD
    map_marker.mesh_resource = 'file:///home/sebastian/ros2ws/output.stl'
    map_marker.scale.x = 1.0
    map_marker.scale.y = 1.0
    map_marker.scale.z = 1.0
    map_marker.color.a = 1.0
    map_marker.color.r = 1.0
    map_marker.color.g = 1.0
    map_marker.color.b = 1.0
    map_marker.pose.position.z = -1.0
    marker_array.markers.append(map_marker)

    # and show the obstacles in red
    map_marker2 = Marker()
    map_marker2.header.frame_id = "odom"
    map_marker2.ns = "map"
    map_marker2.id = 1
    map_marker2.type = Marker.MESH_RESOURCE
    map_marker2.action = Marker.ADD
    map_marker2.mesh_resource = 'file:///home/sebastian/ros2ws/output.steep.stl'
    map_marker2.scale.x = 1.0
    map_marker2.scale.y = 1.0
    map_marker2.scale.z = 1.0
    map_marker2.color.a = 1.0
    map_marker2.color.r = 1.0
    map_marker2.color.g = 0.6
    map_marker2.color.b = 0.6
    map_marker2.pose.position.z = -0.7
    marker_array.markers.append(map_marker2)

    line_pub_.publish(marker_array)

    trial_metadata.inter_cost = result.inter_cost
    trial_metadata.intra_costs = result.intra_costs
    trial_metadata.total_cost = result.total_cost
    total_nodes = 0
    for i in range(len(result.region_order)):
        total_nodes += len(result.intra_paths[result.region_order[i]].data)
        trial_metadata.num_nodes.append(len(result.intra_paths[i].data))
    trial_metadata.num_regions = len(result.region_order)
    if not use_global_tsp:
        trial_metadata.total_nodes = total_nodes

    end_time = clock_.now()
    duration = Duration()
    duration.sec = int((end_time - start_time).nanoseconds // 1e9)
    duration.nanosec = int((end_time - start_time).nanoseconds % 1e9)
    trial_metadata.duration = duration

    metadata_pub_.publish(trial_metadata)

    node_.get_logger().info(f"Calculation finished. Please start the simulator with the generated output.stl and position" \
                            f" the robot at x: {result.intra_paths[result.region_order[0]].data[0].x}, y: {result.intra_paths[result.region_order[0]].data[0].y}." \
                            f" Press any key to continue.")
    input()
    node_.get_logger().warn("Start executing paths. Robot will move now.")
    for i in range(len(result.region_order)):
        node_.get_logger().info(f"Processing region {result.region_order[i]}")
        ground_action_client_.wait_for_server()
        goal = PathAction.Goal()
        path = Path()
        for p in result.intra_paths[result.region_order[i]].data:
            pose = PoseStamped()
            pose.pose.position = p
            path.poses.append(pose)
        goal.goal_path = path
        goal_future = ground_action_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(node_, goal_future)

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            node_.get_logger().warn("Goal rejected")
        else:
            node_.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node_, result_future)
        
        if not i == len(result.region_order):
            node_.get_logger().info(f"Moving to region {i+1}")
            node_.get_logger().info(f"Fly to target height: {max_height + 1.0}")
            goal = FlightAction.Goal()
            goal.level = max_height + 1.0
            goal.landing_height = result.intra_paths[result.region_order[i+1]].data[0].z + 0.1
            goal.speed = flight_speed
            goal.rot_speed = flight_rot_speed
            goal.stay_enabled = False    
            path = Path()        
            pose = PoseStamped()
            pose.pose.position = result.intra_paths[result.region_order[i+1]].data[0]
            path.poses.append(pose)
            goal.goal_path = path
            goal_future = flight_level_action_client_.send_goal_async(goal)
            rclpy.spin_until_future_complete(node_, goal_future)

            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                node_.get_logger().warn("Goal rejected")
            else:
                node_.get_logger().info("Goal accepted")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node_, result_future)



    node_.get_logger().info(f"Finished trial with  {total_nodes} nodes.")
else:
    node_.get_logger().error("Action execution failed")






