#!/usr/bin/env python

# Copy from ros-bridge/carla_ros_scenario_runner/src/carla_ros_scenario_runner

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control that sends route topic from scenario
"""


import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

import threading

from route_planning_msgs.msg import Route, RouteElement, LaneElement
from trajectory_planning_msgs.msg import Trajectory
from perception_msgs.msg import EgoData

import tf2_ros
import carla
import carla_common.transforms as trans


from srunner.scenariomanager.actorcontrols.external_control import ExternalControl  # pylint: disable=import-error


class RosVehicleControlRouteTopic(ExternalControl):

    def __init__(self, actor, args=None):
        super().__init__(actor)

        print(f"RosVehicleControlRouteTopic args: {args}", flush=True)

        params = {}
        params["ego_data_topic"] = "/simulation/ego_data"
        params["trajectory_topic"] = "/planning/drivable_trajectory"
        params["route_topic"] = "/carla_scenario_runner/route"

        if "initial_speed" in args:
            self._initial_speed = float(args["initial_speed"])
            actor.set_target_velocity(carla.Vector3D(self._initial_speed, 0, 0))  

        if "ego_data_topic_name" in args:
            params["ego_data_topic"] = args["ego_data_topic_name"]

        if "trajectory_topic_name" in args:
            params["trajectory_topic"] = args["trajectory_topic_name"]

        if "route_topic_name" in args:
            params["route_topic"] = args["route_topic_name"]

        role_name = actor.attributes["role_name"]

        waypoints = self.get_target_waypoints(args)

        if not rclpy.ok():
            rclpy.init()

        self.node = NavigationClient(role_name, params, waypoints)

        # Run ROS 2 spinning in a separate thread to avoid blocking
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def reset(self):
        pass

    def run_step(self):

        pass

    def get_target_waypoints(self, args):
        """
        function to get waypoints from given arguments from controller
        waypoints are set as properties
        format for a waypoint in openscenario file in "assigncontroller" according to example (srunner/examples/scenariocenter/inD_replay_to_sim_frankenburg_with_controller.xosc):
        <Property name="waypoint_{number}" value="x:1.234,y:2.345,z:0.0,h:3.141592654,p:0.0,r:0.0" />
        """
        waypoint_list = []

        for _, element in args.items():
            if "x:" in element and "y:" in element and "z:" in element:
                x = float(element.split(",")[0].split(":")[1])
                y = -float(element.split(",")[1].split(":")[1])
                z = float(element.split(",")[2].split(":")[1])
                transform = carla.Transform(carla.Location(x, y, z))
                waypoint_list.append(transform)

        return waypoint_list


class NavigationClient(Node):

    def __init__(self, role_name, params, waypoints):
        super().__init__('ros_agent_{}'.format(role_name))

        self.waypoints = waypoints

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

        self.route_pub = self.create_publisher(
            Route,
            params["route_topic"],
            10
        )

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

            self.send_route(self.waypoints)
            self.route_triggered_flag = True


    def send_route(self, waypoints):
        """Generate and publish a route message"""

        print(f"Sending route message", flush=True)

        route = Route()
        route.header.frame_id = "carla_map"
        route.header.stamp = self.time

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

        self.route_pub.publish(route)
