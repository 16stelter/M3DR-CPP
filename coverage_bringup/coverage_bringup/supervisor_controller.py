#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from coverage_bringup.action import SimulateMapAction

class supervisor_controller(Node):
    def __init__(self):
        super().__init__("supervisor_controller")
        ns = self.get_namespace()
        if ns == "/":
            ns = ""

        self._action_server = ActionServer(
            self,
            SimulateMapAction,
            f"{ns}/simulate_map_action",
            self.execute_cb,
            self.handle_goal_cb,
            self.handle_cancel_cb,
        )
        
    def handle_goal_cb(self, goal_request):
        self.get_logger().info("Received goal request")
        return SimulateMapAction.GoalResponse()
    
    def handle_cancel_cb(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return SimulateMapAction.CancelResponse()
    
    def execute_cb(self, goal_handle):
        self.get_logger().info("Executing goal request")
        goal_handle.succeed()
        result = SimulateMapAction.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = supervisor_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
