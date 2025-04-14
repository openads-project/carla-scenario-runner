#!/usr/bin/env python

# Copy from ros-bridge/carla_ros_scenario_runner/src/carla_ros_scenario_runner

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control usable by scenario-runner
"""

import os
import time
import math

import carla 
from srunner.scenariomanager.actorcontrols.external_control import ExternalControl  # pylint: disable=import-error

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_scenario_runner.application_runner import ApplicationRunner

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float64

from route_planning_msgs.msg import Route, RouteElement, LaneElement
from trajectory_planning_msgs.msg import Trajectory

ROS_VERSION = roscomp.get_ros_version()


class RosVehicleControl(ExternalControl):

    def __init__(self, actor, args=None):
        super(RosVehicleControl, self).__init__(actor)

        self.target_waypoints = self.get_target_waypoints(args, actor.get_transform().location) 

        self._carla_actor = actor
        self._role_name = actor.attributes["role_name"]
        if not self._role_name:
            roscomp.logerr("Invalid role_name")

        if "initial_speed" in args:
            self._initial_speed = float(args["initial_speed"])
            self._carla_actor.set_target_velocity(carla.Vector3D(self._initial_speed, 0, 0))  

        self._path_topic_name = "/carla-scenario-runner/path"
        if "path_topic_name" in args:
            self._path_topic_name = args["path_topic_name"]

        self._trajectory_topic_name = "/planning/drivable_trajectory"
        if "trajectory_topic_name" in args:
            self._trajectory_topic_name = args["trajectory_topic_name"]

        self._route_topic_name = "/carla-scenario-runner/route"
        if "path_topic_name" in args:
            self._route_topic_name = args["route_topic_name"]

        roscomp.init("ros_agent_{}".format(self._role_name), args=None)

        self.node = CompatibleNode('ros_agent_{}'.format(self._role_name))

        self._destination_point = None
        self._first_cycle = True

        self._target_speed_publisher = self.node.new_publisher(
            Float64,
            "/carla/{}/target_speed".format(self._role_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing target_speed on /carla/{}/target_speed".format(self._role_name))

        self._path_publisher = self.node.new_publisher(
            Path,
            "/carla/{}/{}".format(self._role_name, self._path_topic_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing path on /carla/{}/{}".format(self._role_name, self._path_topic_name))

        self._route_publisher = self.node.new_publisher(
            Route,
            "/carla/{}/{}".format(self._role_name, self._route_topic_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing route on /carla/{}/{}".format(self._role_name, self._route_topic_name))

        self._destination_publisher = self.node.new_publisher(
            PointStamped,
            "/carla/{}/destination".format(self._role_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing destination on /carla/{}/destination".format(self._role_name))

        self.update_waypoints(self.target_waypoints)


    def controller_runner_log(self, log):  # pylint: disable=no-self-use
        """
        Callback for application logs
        """
        self.node.logwarn("[Controller]{}".format(log))

    def controller_runner_status_updated(self, status):
        """
        Executed from application runner whenever the status changed
        """
        self.node.loginfo("Controller status is: {}".format(status))

    def update_target_speed(self, speed):
        super(RosVehicleControl, self).update_target_speed(speed)
        self.node.loginfo("{}: (ros_vehicle_control) Call update_target_speed function and set speed to {}".format(self._role_name, speed))
        self._target_speed_publisher.publish(Float64(data=speed))

        if self._first_cycle:
            self._carla_actor.set_target_velocity(carla.Vector3D(speed, 0, 0)) 


    def update_waypoints(self, waypoints, start_time=None):
        super(RosVehicleControl, self).update_waypoints(waypoints, start_time)
        self.node.loginfo("{}: (ros_vehicle_control) Call update_waypoints.".format(self._role_name))

        if not self._first_cycle: return

        path = Path()
        path.header.stamp = roscomp.ros_timestamp(sec=self.node.get_time(), from_sec=True)
        path.header.frame_id = "carla_map"

        route = Route()
        route.header.stamp = roscomp.ros_timestamp(sec=self.node.get_time(), from_sec=True)
        route.header.frame_id = "carla_map"

        last_ros_point = None
        s = 0.0
        for wpt in self.target_waypoints:
            ros_point = trans.carla_location_to_ros_point(wpt.location)
            ros_pose = trans.carla_transform_to_ros_pose(wpt)

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

            path.poses.append(PoseStamped(pose=ros_pose))

        # set final route information
        route.destination = trans.carla_location_to_ros_point(self.target_waypoints[-1].location)
        route.remaining_route_elements[-1].lane_elements[-1].has_following_lane_idx = False

        self._path_publisher.publish(path)
        self._route_publisher.publish(route)

        self._destination_point = self.target_waypoints[-1]
        self._first_cycle = False
        self.node.loginfo("Route was published")

        self.node.loginfo("Wait for non-standstill trajectory message ...")
        while (True):
            msg = self.node.wait_for_message(self._trajectory_topic_name, Trajectory)
            if not msg.standstill:
                break
        self.node.loginfo("Received non-standstill trajectory message")

    def reset(self):
        # set target speed to zero before closing as the controller can take time to shutdown

        self.update_target_speed(0.0)

        if self._carla_actor and self._carla_actor.is_alive:
            self._carla_actor = None
        if self._target_speed_publisher:
            self.node.destroy_subscription(self._target_speed_publisher)
            self._target_speed_publisher = None
        if self._path_publisher:
            self.node.destroy_subscription(self._path_publisher)
            self._path_publisher = None
        if self._route_publisher:
            self.node.destroy_subscription(self._route_publisher)
            self._route_publisher = None

    def run_step(self):
        if self._destination_point:
            destination_point = PointStamped()
            destination_point.header.stamp = roscomp.ros_timestamp(sec=self.node.get_time(), from_sec=True)
            destination_point.header.frame_id = "carla_map"
            destination_point.point = trans.carla_location_to_ros_point(self._destination_point.location) 

            self._destination_publisher.publish(destination_point)

    def get_target_waypoints(self, args, al=None):
        """
        function to get waypoints from given arguments from controller
        waypoints are set as properties. 
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
