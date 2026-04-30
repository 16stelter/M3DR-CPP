#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from coverage_path_planning.action import PathAction, FlightAction
from rclpy.executors import MultiThreadedExecutor
import math

class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')

        self.ns = self.get_namespace()
        if self.ns == "/":
            self.ns = ""

        odom_topic = self.declare_parameter("odom_topic", "/true_pose").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        enable_topic = self.declare_parameter("enable_topic", "/velocity_controller/enable").value
        action_topic = self.declare_parameter("action_topic", "/fly_to_goal").value

        self.subscription_odom = self.create_subscription(Odometry, self.ns + odom_topic, self.odom_cb, 10)

        self.cmd_publisher = self.create_publisher(Twist, self.ns + cmd_vel_topic, 10)
        self.enable_publisher = self.create_publisher(Bool, self.ns + enable_topic, 10)

        self.action_server = ActionServer(
            self,
            FlightAction,
            self.ns + action_topic,
            self.flight_action_callback,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.prev_yaw = self.current_yaw
        self.prev_time = self.get_clock().now()

    def flight_action_callback(self, goal_handle):
        result = PathAction.Result()
        msg = Bool()
        msg.data = True
        self.enable_publisher.publish(msg)
        state = 0 # 0: going up, 1: turn, 2: in flight, 3: going down


        while True:
            if goal_handle.is_cancel_requested:
                cmd_vel = Twist()
                cmd_vel.linear.z = 0.0
                self.cmd_publisher.publish(cmd_vel)
                if not goal_handle.request.stay_enabled:
                    msg = Bool()
                    msg.data = False
                    self.enable_publisher.publish(msg)
                result.success = False
                goal_handle.canceled()
                return result
            
            cmd_vel = Twist()
            if state == 0:
                cmd_vel.linear.z = goal_handle.request.speed
                if self.current_z >= goal_handle.request.level:
                    cmd_vel.linear.z = 0.0
                    state = 1
            elif state == 1:
                target_heading = math.atan2(
                    goal_handle.request.goal_path.poses[0].pose.position.y - self.current_y,
                    goal_handle.request.goal_path.poses[0].pose.position.x - self.current_x
                )

                angle_error = target_heading - self.current_yaw
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

                cmd_vel.angular.z = max(-goal_handle.request.rot_speed, min(goal_handle.request.rot_speed, angle_error))
                cmd_vel.linear.z = 0.0
                if abs(angle_error) < 0.1:
                    cmd_vel.angular.z = 0.0
                    state = 2
            elif state == 2:
                dist = math.hypot(
                    self.current_x - goal_handle.request.goal_path.poses[0].pose.position.x,
                    self.current_y - goal_handle.request.goal_path.poses[0].pose.position.y
                )

                target_heading = math.atan2(
                    goal_handle.request.goal_path.poses[0].pose.position.y - self.current_y,
                    goal_handle.request.goal_path.poses[0].pose.position.x - self.current_x
                )
                angle_error = target_heading - self.current_yaw
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

                cmd_vel.angular.z = max(-goal_handle.request.rot_speed, min(goal_handle.request.rot_speed, angle_error))

                speed_scale = math.cos(abs(angle_error))
                cmd_vel.linear.x = goal_handle.request.speed * max(speed_scale, 0.0)

                if dist < 0.2:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    state = 3
            elif state == 3:
                cmd_vel.linear.z = -goal_handle.request.speed
                if self.current_z <= goal_handle.request.landing_height:
                    cmd_vel = Twist()
                    cmd_vel.linear.z = 0.0
                    self.cmd_publisher.publish(cmd_vel)
                    if not goal_handle.request.stay_enabled:
                        msg = Bool()
                        msg.data = False
                        self.enable_publisher.publish(msg)
                    result.success = True
                    goal_handle.succeed()
                    return result
            
            self.cmd_publisher.publish(cmd_vel)


    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()