#!/usr/bin/env python3

"""
Written by dhaval-lad under Apache License, Version 2.0, January 2004. See http://www.apache.org/licenses/ for more details. Modified.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from coverage_path_planning.action import PathAction
from rclpy.executors import MultiThreadedExecutor
import math

# Constants for pure pursuit algorithm
speed = 0.5

def euler_from_quaternion(x, y, z, w):
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

def pure_pursuit(current_x, current_y, current_heading, path, index, rot_limit=0.0):
    done = False
    target_point = path[index]

    target_heading = math.atan2(target_point[1] - current_y,
                                target_point[0] - current_x)

    desired_steering_angle = target_heading - current_heading
    desired_steering_angle = (desired_steering_angle + math.pi) % (2 * math.pi) - math.pi


    final_dist = math.hypot(current_x - target_point[0],
                            current_y - target_point[1])

    if abs(desired_steering_angle) > math.pi / 2 or (rot_limit > 0.0 and abs(desired_steering_angle) > rot_limit):
        v = 0.0
    else:
        speed_scale = math.cos(abs(desired_steering_angle))
        v = speed * max(speed_scale, 0.0)

    if rot_limit > 0.0:
        if desired_steering_angle > rot_limit:
            desired_steering_angle = rot_limit
        elif desired_steering_angle < -rot_limit:
            desired_steering_angle = -rot_limit

    if final_dist < 0.2:
        v = 0.0
        index += 1
        if index >= len(path):
            done = True

    return v, desired_steering_angle * 2, index, done


# ROS2 Node for Pure Pursuit algorithm
class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.ns = self.get_namespace()
        if self.ns == "/":
            self.ns = ""

        odom_topic = self.declare_parameter("odom_topic", "/true_pose").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        action_topic = self.declare_parameter("action_topic", "/follow_path").value

        self.subscription_odom = self.create_subscription(Odometry, self.ns + odom_topic, self.odom_callback, 10)

        self.publisher = self.create_publisher(Twist, self.ns + cmd_vel_topic, 10)

        self.action_server = ActionServer(
            self,
            PathAction,
            self.ns + action_topic,
            self.path_action_callback,
        )

        # State variables
        self.path = None  # Holds the planned path
        self.index = 0  # Index of the current closest point on the path
        self.current_x = 0.0  # Robot's current x-coordinate
        self.current_y = 0.0  # Robot's current y-coordinate
        self.current_yaw = 0.0  # Robot's current orientation (yaw)
        self.previous_first_point = None  # To detect if the path has changed
        self.previous_last_point = None  # To detect if the path has changed

    def path_action_callback(self, goal_handle):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in goal_handle.request.goal_path.poses]
        self.index = 0
        result = PathAction.Result()
        twist = Twist()

        while self.path:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.path = None
                self.get_logger().warn("Path goal canceled.")
                result = PathAction.Result()
                result.success = False
                return result

            # Calculate linear and angular velocities based on the pure pursuit algorithm
            twist.linear.x, twist.angular.z, self.index, done = pure_pursuit(
                self.current_x, self.current_y, self.current_yaw, self.path, self.index, goal_handle.request.rotation_speed_limit
            )

            # Check if the robot has reached the goal
            if done:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                result.success = True
                goal_handle.succeed(result)
                self.get_logger().info("Path goal succeeded.")
                return result

            # Publish the computed control command
            self.publisher.publish(twist)

    # Callback function for the odometry update (to get robot's position and orientation)
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()