#!/usr/bin/env python

# Copy from ros-bridge/carla_ros_scenario_runner/src/carla_ros_scenario_runner

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control that sends route topic from scenario
"""


import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

import threading

from route_planning_msgs.msg import Route, RouteElement, LaneElement
from trajectory_planning_msgs.msg import Trajectory

import tf2_ros
import carla
import carla_common.transforms as trans


from srunner.scenariomanager.actorcontrols.basic_control import BasicControl  # pylint: disable=import-error
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class RosVehicleControlRouteTopic(BasicControl):

    def __init__(self, actor, args=None):
        super().__init__(actor)

        print(f"RosVehicleControlRouteTopic args: {args}", flush=True)

        params = {}
        params["trajectory_topic"] = "/planning/drivable_trajectory"
        params["route_topic"] = "/carla_scenario_runner/route"

        if "initial_speed" in args:
            self._initial_speed = float(args["initial_speed"])
            actor_tf = actor.get_transform()
            target_velocity = actor_tf.transform_vector(carla.Vector3D(self._initial_speed, 0, 0))
            actor.set_target_velocity(target_velocity)

        if "trajectory_topic" in args:
            params["trajectory_topic"] = args["trajectory_topic"]

        if "route_topic" in args:
            params["route_topic"] = args["route_topic"]

        role_name = actor.attributes["role_name"]

        if not rclpy.ok():
            rclpy.init()

        self.node = NavigationClient(role_name, params)
        self.node.get_logger().info(
            f"Route topic client initialized for role '{role_name}' "
            f"(trajectory_topic='{params['trajectory_topic']}', "
            f"route_topic='{params['route_topic']}')"
        )

        # Run ROS 2 spinning in a separate thread to avoid blocking
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()
        self.node.get_logger().info("Started ROS 2 spinning thread for NavigationClient")

    def reset(self):
        pass

    def run_step(self):

        pass

    def update_waypoints(self, waypoints, start_time=None):
        self.node.set_route(waypoints)
        return super().update_waypoints(waypoints, start_time)


class NavigationClient(Node):

    def __init__(self, role_name, params):
        super().__init__('ros_agent_{}'.format(role_name))

        self.route = None

        self.route_triggered_flag = False
        self.transform_timeout = Duration(seconds=0.5)

        self.trajectory_sub = self.create_subscription(
            Trajectory,
            params["trajectory_topic"],
            self.trajectory_callback,
            10
        )

        self.route_pub = self.create_publisher(
            Route,
            params["route_topic"],
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Subscribing to trajectory topic '{self.trajectory_topic}' "
            f"and publishing routes to '{self.route_topic}'"
        )

    def trajectory_callback(self, msg):
        if not CarlaDataProvider.is_scenario_running():
            self.get_logger().warning("Scenario not running, ignoring trajectory update")
            return

        if not self.route:
            self.get_logger().warning("No route available, ignoring trajectory update")
            return

        try:
            self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=self.transform_timeout
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:

            self.get_logger().warning(
                f"Transform from map to base_link not available yet, waiting for carla-its-adapter: {e}"
            )
            return

        if msg.standstill and not self.route_triggered_flag:
            self.get_logger().info("Received non-standstill trajectory, publishing route once")

            self.send_route()
            self.route_triggered_flag = True


    def set_route(self, waypoints):
        """Set the route based on waypoints"""
        if not waypoints:
            self.get_logger().warning("Empty waypoint list provided, route will not be published")
            self.route = None
            return

        self.route = self._build_route_message(waypoints)
        self.route_triggered_flag = False

    def send_route(self):
        """Publish a route message"""
        if not self.route:
            self.get_logger().warning("No route available to publish")
            return

        waypoint_count = len(self.route.remaining_route_elements)
        self.get_logger().info(
            f"Sending route message with {waypoint_count} waypoint(s)"
        )

        self.route_pub.publish(self.route)

    def _build_route_message(self, waypoints):
        """Generate a route message from waypoints"""

        route = Route()
        route.header.frame_id = "carla_map"
        route.header.stamp = Time(sec=0, nanosec=0)

        last_ros_point = None
        s = 0.0
        for waypoint in waypoints:
            ros_point = trans.carla_location_to_ros_point(waypoint.location)
            ros_pose = trans.carla_transform_to_ros_pose(waypoint)

            # calculate s
            if last_ros_point:
                s = s + math.sqrt((ros_point.x - last_ros_point.x)**2 + (ros_point.y - last_ros_point.y)**2)
            last_ros_point = ros_point

            lane_element = LaneElement()
            lane_element.reference_pose = ros_pose
            lane_element.has_following_lane_idx = True

            route_element = RouteElement()
            route_element.s = s
            route_element.lane_elements.append(lane_element)

            route.remaining_route_elements.append(route_element)

        # set final route information
        route.destination = trans.carla_location_to_ros_point(waypoints[-1].location)
        route.remaining_route_elements[-1].lane_elements[-1].has_following_lane_idx = False

        return route
