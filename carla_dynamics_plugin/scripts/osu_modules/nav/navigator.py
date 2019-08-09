"""! does things

@package navigator
"""
#!/usr/bin/env python

from enum import Enum
import carla
import math
import numpy as np

def _is_within_distance_ahead(target_location, current_location, orientation, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

    return d_angle < 90.0

def _compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / norm_target))

    return (norm_target, d_angle)

class Nav(object):

    def __init__(self, vehicle):

        self._vehicle = vehicle
        self._proximity_threshold = 10.0  # meters
        self._roamer = None
        self._world = self._vehicle.get_world()
        self._map = self._vehicle.get_world().get_map()
        self._last_traffic_light = None

        def _is_light_red(self, lights_list):
            """
            Method to check if there is a red light affecting us. This version of
            the method is compatible with both European and US style traffic lights.

            :param lights_list: list containing TrafficLight objects
            :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
            """
            if self._map.name == 'Town01' or self._map.name == 'Town02':
                return self._is_light_red_europe_style(lights_list)
            else:
                return self._is_light_red_us_style(lights_list)

        def _is_light_red_europe_style(self, lights_list):
            """
            This method is specialized to check European style traffic lights.

            :param lights_list: list containing TrafficLight objects
            :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
            """
            ego_vehicle_location = self._vehicle.get_location()
            ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

            for traffic_light in lights_list:
                object_waypoint = self._map.get_waypoint(traffic_light.get_location())
                if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                        object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                    continue

                loc = traffic_light.get_location()
                if _is_within_distance_ahead(loc, ego_vehicle_location,
                                            self._vehicle.get_transform().rotation.yaw,
                                            self._proximity_threshold):
                    if traffic_light.state == carla.TrafficLightState.Red:
                        return (True, traffic_light)

            return (False, None)

        def _is_light_red_us_style(self, lights_list, debug=False):
            """
            This method is specialized to check US style traffic lights.

            :param lights_list: list containing TrafficLight objects
            :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
            """
            ego_vehicle_location = self._vehicle.get_location()
            ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

            if ego_vehicle_waypoint.is_intersection:
                # It is too late. Do not block the intersection! Keep going!
                return (False, None)

            if self._local_planner.target_waypoint is not None:
                if self._local_planner.target_waypoint.is_intersection:
                    min_angle = 180.0
                    sel_magnitude = 0.0
                    sel_traffic_light = None
                    for traffic_light in lights_list:
                        loc = traffic_light.get_location()
                        magnitude, angle = _compute_magnitude_angle(loc,
                                                                ego_vehicle_location,
                                                                self._vehicle.get_transform().rotation.yaw)
                        if magnitude < 60.0 and angle < min(25.0, min_angle):
                            sel_magnitude = magnitude
                            sel_traffic_light = traffic_light
                            min_angle = angle

                    if sel_traffic_light is not None:
                        if debug:
                            print('=== Magnitude = {} | Angle = {} | ID = {}'.format(
                                sel_magnitude, min_angle, sel_traffic_light.id))

                        if self._last_traffic_light is None:
                            self._last_traffic_light = sel_traffic_light

                        if self._last_traffic_light.state == carla.TrafficLightState.Red:
                            return (True, self._last_traffic_light)
                    else:
                        self._last_traffic_light = None

            return (False, None)

        def _is_vehicle_hazard(self, vehicle_list):
            """
            Check if a given vehicle is an obstacle in our way. To this end we take
            into account the road and lane the target vehicle is on and run a
            geometry test to check if the target vehicle is under a certain distance
            in front of our ego vehicle.

            WARNING: This method is an approximation that could fail for very large
            vehicles, which center is actually on a different lane but their
            extension falls within the ego vehicle lane.

            :param vehicle_list: list of potential obstacle to check
            :return: a tuple given by (bool_flag, vehicle), where
                    - bool_flag is True if there is a vehicle ahead blocking us
                    and False otherwise
                    - vehicle is the blocker object itself
            """

            ego_vehicle_location = self._vehicle.get_location()
            ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

            for target_vehicle in vehicle_list:
                # do not account for the ego vehicle
                if target_vehicle.id == self._vehicle.id:
                    continue

                # if the object is not in our lane it's not an obstacle
                target_vehicle_waypoint = self._map.get_waypoint(target_vehicle.get_location())
                if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                        target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                    continue

                loc = target_vehicle.get_location()
                if _is_within_distance_ahead(loc, ego_vehicle_location,
                                            self._vehicle.get_transform().rotation.yaw,
                                            self._proximity_threshold):
                    return (True, target_vehicle)

            return (False, None)