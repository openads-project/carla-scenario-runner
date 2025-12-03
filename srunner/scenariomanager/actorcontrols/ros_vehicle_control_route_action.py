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
from perception_msgs.msg import EgoData

import tf2_ros
import carla

from srunner.scenariomanager.actorcontrols.external_control import ExternalControl  # pylint: disable=import-error


class RosVehicleControlRouteAction(ExternalControl):

    def __init__(self, actor, args=None):
        super().__init__(actor)

        print(f"RosVehicleControlRouteAction args: {args}", flush=True)

        params = {}
        params["ego_data_topic"] = "/simulation/ego_data"
        params["trajectory_topic"] = "/planning/drivable_trajectory"
        params["route_action"] = "/planning/lanelet2_route_planning/plan_route"

        if "initial_speed" in args:
            self._initial_speed = float(args["initial_speed"])
            actor_tf = actor.get_transform()
            target_velocity = actor_tf.transform_vector(carla.Vector3D(self._initial_speed, 0, 0))
            actor.set_target_velocity(target_velocity)

        if "ego_data_topic_name" in args:
            params["ego_data_topic"] = args["ego_data_topic_name"]

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

        # Run ROS 2 spinning in a separate thread to avoid blocking
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

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
        self.initialized_position = False
        self.time = Time(sec=0, nanosec=0)

        self.trajectory_sub = self.create_subscription(
            Trajectory,
            params["trajectory_topic"],
            self.trajectory_callback,
            10
        )

        self.egodata_sub = self.create_subscription(
            EgoData,
            params["ego_data_topic"],
            self.egodata_callback,
            10
        )

        self.clock_sub = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.route_action_client = ActionClient(self, PlanRoute, params["route_action"])

        # wait for the action server to be available
        self.route_action_client.wait_for_server()

    def clock_callback(self, msg):
        self.time = msg.clock

    def egodata_callback(self, msg):
        if not self.initialized_position:
            x = msg.state.continuous_state[0]
            y = msg.state.continuous_state[1]

            # default launch position is at (x,y) = (1000, 1000)
            if x < 995 or x > 1005 or y < 995 or y > 1005:
                self.initialized_position = True
                print(f"Initialized position at ({x}, {y})", flush=True)

    def trajectory_callback(self, msg):
        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:

            print(f"Transform from map to base_link not exist, wait for carla-its-adapter: {e}", flush=True)
            return

        if msg.standstill and not self.route_triggered_flag:
            print("Received first not standstill trajectory ...", flush=True)

            self.call_action(self.target_x, self.target_y, yaw=0.0)
            self.route_triggered_flag = True

    def call_action(self, x, y, yaw):
        """Send a navigation goal to the action server"""

        print(f"Sending goal destination ({x}, {y}) with yaw {yaw}", flush=True)

        point_map = PointStamped()
        point_map.header.frame_id = "carla_map"
        point_map.header.stamp = self.time
        point_map.point = Point(x=x, y=y, z=0.0)

        goal_msg = PlanRoute.Goal()
        goal_msg.destination = point_map

        send_goal_future = self.route_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.action_response_callback)

    def action_response_callback(self, future):
        """Called when the goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected :(", flush=True)
            self.route_triggered_flag = False
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
