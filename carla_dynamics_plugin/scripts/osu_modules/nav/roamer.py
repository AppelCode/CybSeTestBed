#!/usr/bin/env python
# based on Carla roaming agent class
# removed waste

"""! does things

@package roamer
"""

from planner import Plan
from enum import Enum

class _AgentState(Enum):
    """
    AGENT_STATE represents the possible states of a roaming agent
    """
    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_RED_LIGHT = 3

class Roamer(Plan):

    def __init__(self, vehicle):
        super(Roamer,self).__init__(self,vehicle)
        self._state = _AgentState.NAVIGATING

    def _check_hazards(self,vehicle_list,light_list):
        debug = False
        hazard_detected = False
        # check possible obstacles
        vehicle_state, vehicle = super(Roamer,self)._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))

            self._state = _AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

                # check for the state of the traffic lights
        light_state, traffic_light = super(Roamer,self)._is_light_red(light_list)
        if light_state:
            if debug:
                print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))

            self._state = _AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True

        return hazard_detected

    def roamer_run_step(self):

        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        light_list = actor_list.filter("*traffic_light*")

        hazard = self._check_hazards(vehicle_list,light_list)
        target_waypoint_buffer = super(Roamer,self).run_step()

        return hazard, target_waypoint_buffer



