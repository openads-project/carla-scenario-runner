#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module holds the execution state that controllers and the scenario runner
share: whether the ScenarioManager is ticking, and whether a ROS route action
client reported its result.
"""

import threading


class ScenarioState(object):

    """
    Execution state shared between the ScenarioManager, the actor controllers
    and the scenario runner. It is independent of the simulator connection,
    which lives in SimulationData.
    """

    _scenario_running = False
    _route_action_registered = False
    _route_action_completed = False
    _lock = threading.Lock()

    @staticmethod
    def set_scenario_running(flag):
        """
        Mark whether a scenario is currently running. Exposed so controllers can
        defer actions until the ScenarioManager has started ticking.
        """
        ScenarioState._scenario_running = flag

    @staticmethod
    def is_scenario_running():
        """
        @return true if a scenario is currently running
        """
        return ScenarioState._scenario_running

    @staticmethod
    def register_route_action_client():
        """
        Register a ROS route action client that should report completion.
        """
        with ScenarioState._lock:
            ScenarioState._route_action_registered = True
            ScenarioState._route_action_completed = False

    @staticmethod
    def mark_route_action_completed():
        """
        Mark a ROS route action client as completed.
        """
        with ScenarioState._lock:
            ScenarioState._route_action_completed = True

    @staticmethod
    def route_action_registered():
        """
        @return true if any route action client is registered for this scenario
        """
        with ScenarioState._lock:
            return ScenarioState._route_action_registered

    @staticmethod
    def route_action_completed():
        """
        @return true if the registered route action client completed
        """
        with ScenarioState._lock:
            return ScenarioState._route_action_completed

    @staticmethod
    def route_action_pending():
        """
        @return true if a route action client is registered and did not complete
        """
        with ScenarioState._lock:
            return ScenarioState._route_action_registered and not ScenarioState._route_action_completed

    @staticmethod
    def reset_route_action_tracking():
        """
        Reset route action tracking between scenarios.
        """
        with ScenarioState._lock:
            ScenarioState._route_action_registered = False
            ScenarioState._route_action_completed = False

    @staticmethod
    def cleanup():
        """
        Reset every entry, so that the next scenario starts from a clean state
        """
        ScenarioState._scenario_running = False
        ScenarioState.reset_route_action_tracking()
