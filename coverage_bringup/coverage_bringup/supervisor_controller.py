#!/usr/bin/env python3

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from coverage_bringup.action import SimulateMapAction
from spline_navigator.action import NavigatePath
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity, DeleteEntity, SetEntityPose
from map_gen.srv import GenerateMap
from coverage_path_planning.action import CoverageAction
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
import os, time



class supervisor_controller(Node):
    def __init__(self, world_name="empty"):
        super().__init__("supervisor_controller")
        ns = self.get_namespace()
        if ns == "/":
            ns = ""

        self.iteration_count = 0

        self._action_server = ActionServer(
            self,
            SimulateMapAction,
            f"{ns}/simulate_map_action",
            self.execute_cb,
            cancel_callback=self.cancel_callback,
        )

        self.spawn_entity_client_ = self.create_client(SpawnEntity, f"{ns}/world/{world_name}/create")
        self.remove_entity_client_ = self.create_client(DeleteEntity, f"{ns}/world/{world_name}/remove")
        self.set_entity_pose_client_ = self.create_client(SetEntityPose, f"{ns}/world/{world_name}/set_pose")
        self.control_world_client_ = self.create_client(ControlWorld, f"{ns}/world/{world_name}/control")

        self.map_gen_client_ = self.create_client(GenerateMap, f"{ns}/map_gen/generate_map")
        self.coverage_client_ = ActionClient(self, CoverageAction, f"{ns}/coverage_action")
        self.nav_client_ = ActionClient(self, NavigatePath, f"{ns}/navigate_path")

        self.line_pub_ = self.create_publisher(MarkerArray, f"{ns}/coverage_marker", 10)


        self.get_logger().info("Supervisor controller initialized")

    def cancel_callback(self, cancel_request):
        self.get_logger().info('Cancelling goal...')
        return CancelResponse.ACCEPT
    
    def execute_cb(self, goal_handle):
        self.get_logger().info("Executing goal request")

        self.goal_handle_ = goal_handle
        self.goal_ = goal_handle.request

        # Call the map generation service
        self.map_gen_client_.wait_for_service()
        request = GenerateMap.Request()
        request.in_file_path = self.goal_.in_filepath
        request.in_file_reduction_factor = self.goal_.in_file_reduction_factor
        request.file_path = os.path.splitext(self.goal_.map_filepath)[0] + str(self.iteration_count) + ".stl"
        request.seed = self.goal_.seed
        request.size = self.goal_.size
        request.max_slope = self.goal_.max_slope
        request.smoothing_iterations = self.goal_.smoothing_iterations
        future = self.map_gen_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Call the coverage action
        self.coverage_client_.wait_for_server()
        coverage_goal = CoverageAction.Goal()
        coverage_goal.map_filepath = os.path.splitext(self.goal_.map_filepath)[0] + str(self.iteration_count) + ".stl"
        coverage_goal.runs = 3
        goal_future = self.coverage_client_.send_goal_async(coverage_goal)
        rclpy.spin_until_future_complete(self, goal_future)
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        coverage_path = result_future.result().result
        
        if coverage_path is not None:
            self.get_logger().info(f"Executing path with total path length: {coverage_path.total_cost}")
            self.get_logger().info(f"Total regions: {len(coverage_path.region_order)}")

            self.publish_rviz_markers(coverage_path)

            self.unpause()

            self.pause(reset=True)
            self.remove_entity('ground_plane')
            self.entity_reload_helper(os.path.splitext(self.goal_.map_filepath)[0], 'map_gen_dummy')

            self.create_entity('ground_plane', 'map_gen_dummy')

            self.iteration_count += 1

            response = self.execute_path(coverage_path)

        if response == 0:
            goal_handle.succeed()
        elif response == 1:
            goal_handle.abort()
        result = SimulateMapAction.Result()
        return result


    def publish_rviz_markers(self, coverage_path):

        inter_marker_ = Marker()
        inter_marker_.header.frame_id = "world"
        inter_marker_.ns = "line"
        inter_marker_.id = 1
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
        delete_marker_.header.frame_id = "world"
        delete_marker_.ns = "line"
        delete_marker_.action = Marker.DELETEALL

        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker_)
        self.line_pub_.publish(marker_array)
        marker_array = MarkerArray()
        inter_marker_.points = []
        for p in range(1, len(coverage_path.inter_path)-1):
            inter_marker_.points.append(coverage_path.inter_path[p])
        marker_array.markers.append(inter_marker_)
        for r in range(len(coverage_path.intra_paths)):
            intra_marker = Marker()
            intra_marker.header.frame_id = "world"
            intra_marker.ns = "line"
            intra_marker.id = r+2
            intra_marker.type = Marker.LINE_STRIP
            intra_marker.action = Marker.ADD
            intra_marker.pose.orientation.w = 1.0
            intra_marker.scale.x = 0.05
            intra_marker.color.r = 0.0
            intra_marker.color.g = 0.0
            intra_marker.color.b = 1.0
            intra_marker.color.a = 1.0
            for p in coverage_path.intra_paths[r].data:
                intra_marker.points.append(p)
            marker_array.markers.append(intra_marker)
        self.line_pub_.publish(marker_array)
        return 0

    def execute_path(self, coverage_path):
        for i in range(len(coverage_path.region_order)):
            self.pause()
            self.set_entity_pose("leo_rover", coverage_path.inter_path[i*2].x, coverage_path.inter_path[i*2].y, coverage_path.inter_path[i*2].z + 0.1)
            self.unpause()
            self.get_logger().info("Starting navigation")
            while rclpy.ok():
                self.nav_client_.wait_for_server()
                goal = NavigatePath.Goal()
                self.get_logger().info(f'{coverage_path.intra_paths[coverage_path.region_order[i]]}')
                goal.path = coverage_path.intra_paths[coverage_path.region_order[i]].data

                goal_future = self.nav_client_.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, goal_future)
                result_future = goal_future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result

                if result.success:
                    self.get_logger().info("Navigation successful")
                    break
                # Break on cancel request
                if self.goal_handle_.is_cancel_requested:
                    self.get_logger().warning("Goal cancelled")
                    self.goal_handle_.canceled()
                    return 2
                rclpy.spin_once(self)
        return 0
    
    def remove_entity(self, entity_name):
        self.get_logger().info(f"Removing entity {entity_name}")
        request = DeleteEntity.Request()
        request.entity.name = entity_name
        request.entity.type = 2
        future = self.remove_entity_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
        return 0
    
    def create_entity(self, entity_name, sdf_filename):
        self.get_logger().info(f"Spawning entity {entity_name} ({sdf_filename})")
        request = SpawnEntity.Request()
        request.entity_factory.name = entity_name
        request.entity_factory.sdf_filename = sdf_filename + str(self.iteration_count) + ".sdf"
        future = self.spawn_entity_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
        return 0
    
    def set_entity_pose(self, entity_name, x, y, z):
        self.get_logger().info(f"Setting pose for entity {entity_name} to ({x}, {y}, {z})")
        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        future = self.set_entity_pose_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
        return 0
    
    def pause(self, reset=False):
        self.get_logger().info("Pausing simulation")
        request = ControlWorld.Request()
        request.world_control.pause = True
        request.world_control.reset.model_only = reset
        request.world_control.step = True
        future = self.control_world_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
        return 0
    
    def unpause(self):
        self.get_logger().info("Unpausing simulation")
        request = ControlWorld.Request()
        request.world_control.pause = False
        request.world_control.step = True
        future = self.control_world_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.1)
        return 0

    def entity_reload_helper(self, stl_filepath, sdf_filename):
        sandbox_path = get_package_share_directory('ros2_gazebo_sandbox')
        if self.iteration_count > 0:
            os.remove(f"{sandbox_path}/worlds/{sdf_filename}{self.iteration_count-1}.sdf")
            os.remove(f"{stl_filepath}{self.iteration_count-1}.stl")
        with open(f"{sandbox_path}/worlds/{sdf_filename}.sdf", 'r') as f:
            template = f.read()

        content = template.replace("{stl_path}", f"{os.path.basename(stl_filepath)}{self.iteration_count}.stl")
        with open(f"{sandbox_path}/worlds/{sdf_filename}{self.iteration_count}.sdf", 'w') as f:
            f.write(content)


def main(args=None):
    rclpy.init(args=args)
    node = supervisor_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
