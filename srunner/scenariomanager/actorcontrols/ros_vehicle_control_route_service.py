#!/usr/bin/env python

# Copy from ros-bridge/carla_ros_scenario_runner/src/carla_ros_scenario_runner

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control that sends route action usable by scenario-runner
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import ros_compatibility as roscomp
import threading

from geometry_msgs.msg import PointStamped, Point
from route_planning_msgs.action import PlanRoute
from trajectory_planning_msgs.msg import Trajectory

import tf2_ros
from tf2_ros import TransformException

from srunner.scenariomanager.actorcontrols.external_control import ExternalControl  # pylint: disable=import-error

ROS_VERSION = roscomp.get_ros_version()


class RosVehicleControlRouteService(ExternalControl):

    def __init__(self, actor, args=None):
        super().__init__(actor)

        print(f"RosVehicleControlRouteService args: {args}", flush=True)
        target_x = float(args["target_x"])
        target_y = float(args["target_y"])

        rclpy.init()
        self.node = NavigationClient()

        # Run ROS 2 spinning in a separate thread to avoid blocking
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def reset(self):
        pass

    def run_step(self):
        pass

class NavigationClient(Node):
    def __init__(self):
        super().__init__("navigation_client")
        print("NavigationClient initialized", flush=True)

        self.received_trajectory = False

        self.trajectory_sub = self.create_subscription(
            Trajectory,
            "/planning/drivable_trajectory",  # oder was dein Topic ist
            self.trajectory_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = ActionClient(self, PlanRoute, '/lanelet2_route_planning/plan_route')

        # Wait for the action server to be available
        self.client.wait_for_server()

    def trajectory_callback(self, msg):
        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time()
            )
        except tf2_ros.LookupException as e:
            print(f"Transform from map to base_link not exist, wait for carla-its-adapter: {e}", flush=True)
            return

        if not msg.standstill and not self.received_trajectory:
            print("Received first not standstill trajectory ...", flush=True)

            self.send_goal(x=target_x, y=target_y, yaw=0.0)

            self.received_trajectory = True

    def send_goal(self, x, y, yaw):
        """Send a navigation goal to the action server"""
        print(f"Sending goal to ({x}, {y}) with yaw {yaw}", flush=True)
        point_map = PointStamped()
        point_map.header.frame_id = "map"
        point_map.header.stamp = self.get_clock().now().to_msg()
        point_map.point = Point(x=x, y=y, z=0.0)

        goal_msg = PlanRoute.Goal()
        goal_msg.destination = point_map

        send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected :(", flush=True)
            return

        print("Goal accepted!", flush=True)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback from the action server"""
        feedback = feedback_msg.feedback
        print(f"Feedback: distance remaining = {feedback.distance_remaining}", flush=True)

    def result_callback(self, future):
        """Called when the goal is completed"""
        result = future.result().result
        print(f"Goal completed with status: {result}", flush=True)
        rclpy.shutdown()
