#!/usr/bin/env python

# Copy from ros-bridge/carla_ros_scenario_runner/src/carla_ros_scenario_runner

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control that sends route action from scenario
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

import threading

from geometry_msgs.msg import PointStamped, Point
from route_planning_msgs.action import PlanRoute
from trajectory_planning_msgs.msg import Trajectory

import tf2_ros
import carla

from srunner.scenariomanager.actorcontrols.external_control import ExternalControl  # pylint: disable=import-error
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class RosVehicleControlRouteAction(ExternalControl):

    def __init__(self, actor, args=None):
        super().__init__(actor)

        print(f"RosVehicleControlRouteAction args: {args}", flush=True)

        params = {}
        params["trajectory_topic"] = "/planning/drivable_trajectory"
        params["route_action"] = "/planning/lanelet2_route_planning/plan_route"

        if "initial_speed" in args:
            self._initial_speed = float(args["initial_speed"])
            actor_tf = actor.get_transform()
            target_velocity = actor_tf.transform_vector(carla.Vector3D(self._initial_speed, 0, 0))
            actor.set_target_velocity(target_velocity)

        if "trajectory_topic_name" in args:
            params["trajectory_topic"] = args["trajectory_topic_name"]

        if "route_action_name" in args:
            params["route_action"] = args["route_action_name"]

        role_name = actor.attributes["role_name"]

        target_x = float(args["target_x"])
        target_y = float(args["target_y"])

        if not rclpy.ok():
            rclpy.init()

        self.node = NavigationClient(role_name, params, target_x, target_y)
        self.node.get_logger().info(
            f"Route action client initialized for role '{role_name}' "
            f"(target=({target_x:.2f}, {target_y:.2f}), "
            f"trajectory_topic='{params['trajectory_topic']}', "
            f"route_action='{params['route_action']}')"
        )

        # Run ROS 2 spinning in a separate thread to avoid blocking
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()
        self.node.get_logger().info("Started ROS 2 spinning thread for NavigationClient")

    def reset(self):
        pass

    def run_step(self):

        pass


class NavigationClient(Node):
    def __init__(self, role_name, params, target_x, target_y):
        super().__init__('ros_agent_{}'.format(role_name))

        self.target_x = target_x
        self.target_y = target_y

        self.route_triggered_flag = False
        self.initialized_position = True

        self.trajectory_sub = self.create_subscription(
            Trajectory,
            params["trajectory_topic"],
            self.trajectory_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.route_action_client = ActionClient(self, PlanRoute, params["route_action"])

        # wait for the action server to be available
        self.get_logger().info(
            f"Subscribing to trajectory topic '{params['trajectory_topic']}' "
            f"and waiting for action server '{params['route_action']}'"
        )
        self.route_action_client.wait_for_server()
        self.get_logger().info(f"Route action server '{params['route_action']}' available")

    def trajectory_callback(self, msg):
        if not CarlaDataProvider.is_scenario_running():
            self.get_logger().warning("Scenario not running, ignoring trajectory update")
            return

        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:

            self.get_logger().warning(
                f"Transform from map to base_link not available yet, waiting for carla-its-adapter: {e}"
            )
            return

        if self.initialized_position and msg.standstill and not self.route_triggered_flag:
            self.get_logger().info("Received first non-standstill trajectory, triggering route action")

            self.call_action(self.target_x, self.target_y, yaw=0.0)
            self.route_triggered_flag = True

    def call_action(self, x, y, yaw):
        """Send a navigation goal to the action server"""

        self.get_logger().info(f"Sending goal destination ({x}, {y}) with yaw {yaw:.2f}")

        point_map = PointStamped()
        point_map.header.frame_id = "carla_map"
        point_map.header.stamp = Time(sec=0, nanosec=0)
        point_map.point = Point(x=x, y=y, z=0.0)

        goal_msg = PlanRoute.Goal()
        goal_msg.destination = point_map

        send_goal_future = self.route_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.action_response_callback)

    def action_response_callback(self, future):
        """Called when the goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Route action goal rejected")
            self.route_triggered_flag = False
            return

        self.get_logger().info("Route action goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Route action feedback: distance remaining = {feedback.distance_remaining}"
        )

    def result_callback(self, future):
        """Called when the goal is completed"""
        result = future.result().result
        self.get_logger().info(f"Route action goal completed with status: {result}")
        self.get_logger().info("Shutting down rclpy after route action completion")
        rclpy.shutdown()
