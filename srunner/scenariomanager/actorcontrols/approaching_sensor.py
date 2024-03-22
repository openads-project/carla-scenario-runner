#!/usr/bin/env python
# Copyright (c) 2020-2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
This module provides an control for vehicles following a predefined path and breaking for vehicles in front of it 
(no matter if it is on an intersection or not)

There is no sensor included, but ground trouth data is used 
"""
from distutils.util import strtobool
import math
import carla
import numpy as np
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import matplotlib.pyplot as plt
import time

from shapely.geometry import Polygon


class ApproachingSensor(BasicControl):

    def __init__(self, actor, args=None):
        super(ApproachingSensor, self).__init__(actor)
        
        self.ttc_values = []
        self.target_waypoints = self.get_target_waypoints(args, actor.get_transform().location) # via location only for scenario.center paper scenarios
        self.is_plotted = False
        self.reached_waypoint_index = 0
        self.driving_state = []
        
        self.moving_object_ids = []
        
        if "direct_evaluation" in args.keys() and args["direct_evaluation"] == "True":
            self.evaluation = True
        else:
            self.evaluation = False
        
        # just for debugging
        self.last_picture_made = 0
        
    def get_target_waypoints(self, args, al=None):
        """
        function to get waypoints from given arguments from controller
        waypoints are set as properties. each waypoint is one line. format: "x:1.234,y:3.345,z:6.423"
        """
        
        waypoint_list = []
            
        for _, element in args.items():
            if "x:" in element and "y:" in element and "z:" in element:
                x = float(element.split(",")[0].split(":")[1])
                y = -float(element.split(",")[1].split(":")[1])
                z = float(element.split(",")[2].split(":")[1])
                waypoint_list.append(carla.Vector3D(x=x, y=y, z=z))
        return waypoint_list
        
    def reset(self):
        """
        Reset the controller
        """
        pass
    
    def run_step(self, debug=False):
        """
        Execute on tick of the controller's control loop
        If _waypoints are provided, the vehicle moves towards the next waypoint
        with the given _target_speed, until reaching the final waypoint. Upon reaching
        the final waypoint, _reached_goal is set to True.
        If _waypoints is empty, the vehicle moves in its current direction with
        the given _target_speed.
        For further details see :func:`_set_new_velocity`
        """       
        # try and except needed since vehicle may disappear - in this case, no update is needed
        try:
            ego_transform = self._actor.get_transform()
            ego_location = ego_transform.location
            ego_location.z = ego_location.z if ego_location.z < 1.0 else 0.2 
            ego_velocity = self._actor.get_velocity()
            ego_velocity_abs = self._calc_velocity(ego_velocity)
        except :
            # save ttc function
            if not self.is_plotted and self.evaluation:
                self._print_ttcs()
            return
        
        target_waypoint = self._calculate_actual_target_waypoint(ego_location, lookahead_points=10)
        target_velocity = 70.0/3.6
        
        
        """ set rotation and velocity direction"""
        # set rotation
        set_rotation = carla.Rotation(roll=0, pitch=0, yaw=np.arctan2(target_waypoint.y-ego_location.y, target_waypoint.x-ego_location.x)/math.pi*180 % 360)
        self._actor.set_transform(carla.Transform(ego_location, set_rotation))
        
        # set velocity vector according to rotation
        vel_abs = self._calc_velocity(self._actor.get_velocity())
        yaw_rad = math.radians(set_rotation.yaw)
        velocity_vector = carla.Vector3D(x=vel_abs*(math.cos(yaw_rad)), 
                                         y=vel_abs*(math.sin(yaw_rad)), 
                                         z=0)
        self._actor.set_target_velocity(velocity_vector)
        
        """ from here acceleration/ deceleration """
        ttc, ru_ttc = self._calc_advanced_ttx_metric(metric_type="ttc")
        thw, ru_thw = self._calc_advanced_ttx_metric(metric_type="thw")

        self.ttc_values.append(ttc)
                
        # simulate that randomly vehicle is not detected
        import random
        if random.randint(0, 100) > 0:
            is_detected = True
        else:
            is_detected = False
                
        if ((ttc < 1.5 and thw < 1.5) or thw < 0.5) and is_detected:
            self._actor.apply_control(carla.VehicleControl(throttle = 0, brake = 1.0))
            self.driving_state.append(0)
        else:
            if ego_velocity_abs > target_velocity:
                self._actor.apply_control(carla.VehicleControl(throttle = 0, brake = 0))
                self.driving_state.append(0.5)
            else:
                self._actor.apply_control(carla.VehicleControl(throttle = 1, brake = 0))
                self.driving_state.append(1.0)
            
    def _print_ttcs(self, path="/tmp/scenario-center_simulations/"):
        """
        print ttc values over time including minimum values
        """
        print("Min ttc: %.4f" % min(self.ttc_values[15:]))
        
        if not hasattr(self, "env_scenario"):
            self.env_scenario = "na"
        
        fig, ax = plt.subplots(1)
        ax.plot(self.ttc_values)
        ax.plot(self.driving_state)
        ax.set_title(str(min(self.ttc_values[15:])))
        fig.savefig(path + "plot_ttc_env" + str(self.env_scenario) + "_" + str(time.time())+".png")
        print("SAVED DATA for env " + str(self.env_scenario))
        self.is_plotted = True
        
    def _calc_velocity(self, vel):
            return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        
    def _calculate_actual_target_waypoint(self, ego_location, lookahead_points=0):
        """
        get actual nearest waypoint and returns target (with potential lookahead)
        """
        nearest_waypoint_index = None
        actual_distance = math.inf
        
        wp_index = -1
        
        if self.reached_waypoint_index > len(self.target_waypoints)-2:
            nearest_waypoint_index = len(self.target_waypoints)-1
        else:
            initial_correction = self.reached_waypoint_index+1
            for index, waypoint in enumerate(self.target_waypoints[initial_correction:]):
                corrected_index = index + initial_correction
                
                # if not reached, check actual distance
                dist = math.sqrt((ego_location.x-waypoint.x)**2 + (ego_location.y-waypoint.y)**2 + (ego_location.z-waypoint.z)**2)
                if dist > actual_distance:
                    nearest_waypoint_index = corrected_index
                    break
                else:
                    actual_distance = dist
                    self.reached_waypoint_index += 1
                
        if not nearest_waypoint_index:
            wp_index = len(self.target_waypoints)-1
        else:
            wp_index = min(nearest_waypoint_index+lookahead_points, len(self.target_waypoints)-1)
        return self.target_waypoints[wp_index]
    
    def _calculate_ttc_simplified(self):
        """
        ttc calculation with simplifications - e.g. not working if road users are turning/ not directly in front
        """
        ego_transform = self._actor.get_transform()
        ego_rotation = ego_transform.rotation
        ego_location = ego_transform.location
        ego_velocity = self._actor.get_velocity()
        ego_velocity_abs = self._calc_velocity(ego_velocity)
        
        ttc = math.inf
        vehicles_same_direction = 0
        considered_vehicles = 0
        
        actors = CarlaDataProvider.get_actors()
        for key, actor in list(actors):
            obj_location = actor.get_transform().location
            obj_rotation = actor.get_transform().rotation
            obj_velocity = actor.get_velocity()
            
            # skip calculation for pedestrians - only consider VRUs, but no pedestrians on road
            if "walker" in actor.type_id:
                continue

            if abs(ego_rotation.yaw - obj_rotation.yaw) < 15:
                vehicles_same_direction += 1
                # check if ahead
                if self._calc_velocity(obj_velocity) >= ego_velocity_abs:
                    continue
                if self._calc_velocity(obj_velocity) < 0.5:
                    continue
                if np.dot(np.array([ego_velocity.x, ego_velocity.y, ego_velocity.z]), np.array([obj_location.x-ego_location.x, obj_location.y-ego_location.y, obj_location.z-ego_location.z])) > 0:
                    distance = math.sqrt((ego_location.x-obj_location.x)**2+(ego_location.y-obj_location.y)**2+(ego_location.z-obj_location.z)**2)
                    delta_velocities = math.sqrt((ego_velocity.x-obj_velocity.x)**2+(ego_velocity.y-obj_velocity.y)**2+(ego_velocity.z-obj_velocity.z)**2)
                    ttc_opt = (distance) / (delta_velocities)
                    ttc = min(ttc, ttc_opt)
                    considered_vehicles += 1
        return ttc
   
    def _calc_distance(self, loc1, loc2):
        return math.sqrt((loc1.x-loc2.x)**2 + (loc1.y-loc2.y)**2 + (loc1.z-loc2.z)**2)
    
    def _calc_advanced_ttx_metric(self, metric_type, discretization=0.05, time_horizon=5.0):
        """
        calculation of a given ttx metic
        available: thw, ttc
        calculation for intersections (based on ego path and linear extrapolation of object movement)
        """
        # check if metric type available
        if metric_type not in ["thw", "ttc"]:
            print("ERROR: No such metric type '"+metric_type+"' available.")            
            return None, None
        
        ttx = math.inf
        considered_time_horizon_in_seconds = time_horizon
        
        ru_type = None
        
        # ego movement prediction based on polyline and actual velocity
        ego_velocity_abs = self._calc_velocity(self._actor.get_velocity())
        if self.reached_waypoint_index > len(self.target_waypoints)-1:
            return ttx # then there is to less information to predict
        predicted_ego_state = self._predict_ego_state(ego_velocity_abs, considered_time_horizon_in_seconds, discretization)
        
        for object_ru_item in CarlaDataProvider.get_actors():
            ttx_to_ru = math.inf
            
            # get actual ru state
            object_ru = object_ru_item[1]
            object_ru_id = object_ru_item[0]
            object_transform = object_ru.get_transform()
            object_velocity = object_ru.get_velocity()
            object_velocity_abs = self._calc_velocity(object_velocity)
            
            # font check against ego road user
            if object_ru == self._actor:
                continue
            
            # filter potentially irrelevant rus (always standing as observed) 
            # WARNING: this include objects always parking although they may be on street. Simplification done for calculation efficiency 
            if object_velocity_abs < 0.01 and object_ru_id not in self.moving_object_ids:
                continue
            
            if object_ru_id not in self.moving_object_ids:
                self.moving_object_ids.append(object_ru_id)
            
            if metric_type == "ttc":
                project_object_state = True
            elif metric_type == "thw":
                project_object_state = False
            
            # extrapolate state x seconds (according to velocity vector - without taking infrastructure into account)
            for index, timestep in enumerate(np.arange(0, considered_time_horizon_in_seconds, discretization)):
                # check whether prediction exists for ego state (or is out of range)
                if len(predicted_ego_state) > index:  # check if prediction is available or cannot be made (e.g. because of significant extrapolation)
                    distance, can_be_reached = self._check_projected_distance(predicted_ego_state[index], ego_vel_abs=ego_velocity_abs, object_ru=object_ru, object_transform=object_transform, object_velocity=object_velocity, object_abs_velocity=object_velocity_abs, timestep=timestep, max_time=considered_time_horizon_in_seconds, project_object=project_object_state)
                    if not can_be_reached:
                        break
                    if distance == 0:
                        ttx_to_ru = timestep
                        break
                
            # check whether it is the smallest 
            if ttx_to_ru < ttx:
                ru_type = object_ru.type_id
            ttx = min(ttx, ttx_to_ru)
        return ttx, ru_type
    
    def _check_projected_distance(self, predicted_ego_state, ego_vel_abs, object_ru, object_transform, 
                                  object_velocity, object_abs_velocity, timestep, max_time, project_object=True):
        approx_distance = math.inf
        can_be_reached = True
        
        object_rotation = object_transform.rotation
        object_location = object_transform.location
        
        if project_object:
            length = timestep
        else:
            length = 0.0
        
        predict_object_location = carla.Vector3D(x=object_location.x + length * object_velocity.x, 
                                                 y=object_location.y + length * object_velocity.y, 
                                                 z=object_location.z + length * object_velocity.z)
        
        # check first approximated distance whether it make sense to include a more accurate calculation
        approx_distance = self._calc_distance(predict_object_location, predicted_ego_state["location"])
        
        # check if can be reached (really rough estimation - can be improved if necessary)
        if (object_abs_velocity + ego_vel_abs) * (max_time-timestep) > approx_distance:
            can_be_reached = True
        else:
            can_be_reached = False
        
        approx_bb_ego = max(2, predicted_ego_state["bb"].length / 4) # divided by 4 because it is circumfence and only half of length and width is needed
        approx_bb_object = max(2, object_ru.bounding_box.extent.x + object_ru.bounding_box.extent.y)
        if approx_distance < (approx_bb_ego + approx_bb_object):
            rect_1 = self._get_bb_shapely(predict_object_location, object_rotation, object_ru.bounding_box.extent.x, object_ru.bounding_box.extent.y)
            if rect_1.intersects(predicted_ego_state["bb"]):
                approx_distance = 0
        return approx_distance, can_be_reached
    
    def _get_bb_shapely(self, location, rotation, length, width):
        # check if length and width is feasible (is done since carla bounding boxes for bicyclist have width and length 0):
        if length == 0:
            length = 1.5
        if width == 0:
            width = 0.3
        
        # Get the corners of the bounding box
        corners = [
            carla.Location(x=length, y=width),
            carla.Location(x=-length, y=width),
            carla.Location(x=-length, y=-width),
            carla.Location(x=length, y=-width),
        ]

        # Rotate the corners according to the actor's rotation
        rad_yaw = np.deg2rad(rotation.yaw)
        rotated_corners = [
            carla.Location(
                x=c.x * np.cos(rad_yaw) - c.y * np.sin(rad_yaw),
                y=c.x * np.sin(rad_yaw) + c.y * np.cos(rad_yaw)
            ) + location
            for c in corners
        ]

        return Polygon([[rotated_corners[0].x, rotated_corners[0].y],
                        [rotated_corners[1].x, rotated_corners[1].y],
                        [rotated_corners[2].x, rotated_corners[2].y],
                        [rotated_corners[3].x, rotated_corners[3].y]
            
        ])
        
    def _predict_ego_state(self, ego_velocity_abs, considered_time_horizon_in_seconds, discretization, offset=0.0):
        """
        predict ego state since there may be curves
        """
        predicted_ego_state = []
        
        last_wp_idx = self.reached_waypoint_index
        abs_length = 0
        
        # discretization of timesteps to assign the correct locatino for each of those
        for index_time_discretization, timestep in enumerate(np.arange(0.0, considered_time_horizon_in_seconds, discretization)):
            
            location = None
            
            # distance to cover (ego drives in a certain timestep according to velocity)
            extrapolated_distance = ego_velocity_abs * timestep
            
            for index_wp_go_through, upcomming_wp in enumerate(self.target_waypoints[last_wp_idx+1:]):
                last_wp_idx_cand = last_wp_idx+index_wp_go_through
                last_wp = self.target_waypoints[last_wp_idx_cand]
                d_length = self._calc_distance(upcomming_wp, last_wp)
                
                if abs_length + d_length <= extrapolated_distance:
                    # distance not reached -> go to next wp
                    abs_length += d_length
                else:
                    # calculate values
                    # location in this region
                    scale = (extrapolated_distance-abs_length) / d_length
                    dx = upcomming_wp.x-last_wp.x
                    dy = upcomming_wp.y-last_wp.y
                    location = carla.Vector3D(x=last_wp.x + scale * dx, 
                                            y=last_wp.y + scale * dy)
                    rotation = carla.Rotation(yaw=180/math.pi*np.arctan2(dy, dx))
                    
                    # save new last waypoint
                    last_wp_idx = last_wp_idx_cand
                    break
            if location:
                predicted_ego_state.append({"bb": self._get_bb_shapely(location, rotation, 
                                                                    length=self._actor.bounding_box.extent.x+offset, 
                                                                    width=self._actor.bounding_box.extent.y+offset), 
                                            "location": location, "rotation": rotation})
        return predicted_ego_state
        