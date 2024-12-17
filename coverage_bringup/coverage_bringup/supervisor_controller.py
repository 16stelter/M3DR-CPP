#!/usr/bin/env python3

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from coverage_bringup.action import SimulateMapAction
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity, DeleteEntity, SetEntityPose
from map_gen.srv import GenerateMap
from coverage_path_planning.action import CoverageAction



class supervisor_controller(Node):
    def __init__(self, world_name="empty"):
        super().__init__("supervisor_controller")
        self.node_ = Node("supervisor_controller")
        ns = self.get_namespace()
        if ns == "/":
            ns = ""

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
        self.coverage_client_ = ActionClient(self.node_, CoverageAction, f"{ns}/coverage_action")

        self.node_.get_logger().info("Supervisor controller initialized")

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
        request.file_path = self.goal_.map_filepath
        request.seed = self.goal_.seed
        request.size = self.goal_.size
        request.max_slope = self.goal_.max_slope
        request.smoothing_iterations = self.goal_.smoothing_iterations
        future = self.map_gen_client_.call_async(request)
        rclpy.spin_until_future_complete(self.node_, future)

        # Call the coverage action
        self.coverage_client_.wait_for_server()
        coverage_goal = CoverageAction.Goal()
        coverage_goal.map_filepath = self.goal_.map_filepath
        coverage_goal.runs = 3
        goal_future = self.coverage_client_.send_goal_async(coverage_goal)
        rclpy.spin_until_future_complete(self.node_, goal_future)
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node_, result_future)
        coverage_path = result_future.result().result
        
        if coverage_path is not None:
            self.node_.get_logger().info(f"Total path length: {coverage_path.total_cost}")

            self.publish_rviz_markers(coverage_path)

            response = self.execute_path(coverage_path)

        if response == 0:
            goal_handle.succeed()
        elif response == 1:
            goal_handle.abort()
        result = SimulateMapAction.Result()
        return result


    def publish_rviz_markers(self, coverage_result):
        return 0

    def execute_path(self, coverage_result):
        while True:
            # Break on cancel request
            if self.goal_handle_.is_cancel_requested:
                self.node_.get_logger().warning("Goal cancelled")
                self.goal_handle_.canceled()
                return 2
            rclpy.spin_once(self.node_)
        return 0

def main(args=None):
    rclpy.init(args=args)
    node = supervisor_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
