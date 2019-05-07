#!/usr/bin/env python

######################################################################################
# Setup for carla 
import glob
import os
import sys

try:
    sys.path.append(glob.glob('/home/matt/Documents/CARLA/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

######################################################################################

import rospy
import numpy as np
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error

import carla
from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner

import argparse

######################################################################################
# AV class 
######################################################################################

#define class for autonomous vehicle
#based on carla roaming_agent.py
#added functionality to allow dynamics and control outisde of carla through ROS
class AV(Agent):
    def __init__(self,vehicle):
        super(AV, self).__init__(vehicle)
        self._proximity_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        self.vehicle_state = None

        #create planner based on environment
        self._local_planner = LocalPlanner(self._vehicle)

    def update_status(self,vehicle_status):
        self.vehicle_state = vehicle_status

######################################################################################
# ROS Classes
######################################################################################

# class for ros input information
class InputMessage:
    def __init__(self):
        self.current_speed = None
        self.current_steering_angle = None

#class for ros output info
class OutputMessage:
    def __init__(self):
        self.setpoint_speed = None
        self.setpoint_angle = None

#create wrapper for messages related to a vehicle
class ROSWrapperForAV:
    def ___int__(self,role_name,controlled_vehicle):
        self.input_message = InputMessage()
        self.output_message = OutputMessage()
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle = controlled_vehicle

        self.vehicle_info_sub = rospy.Subscriber("/carla/" + role_name + "/ackermann_cmd", 
            AckermannDrive, self.update_input)

    def update_input(self, vehicle_status):
        self.vehicle.update_status(vehicle_status)

######################################################################################
# main
######################################################################################     

if __name__ == '__main__':
    #create argument parser
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--carid','vid',
        metavar = 'ID',
        default = 0,
        help = 'enter id of vehicle to control'
    )
    argparser.add_argument(
        '--newcar',
        metavar = 'N',
        default = 0,
        help = 'create new car = 1, use car in world =1 \
            (must provide id or new car will be created)'
    )
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)'
    )
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)'
    )
    args = argparser.parse_args()

    #connect to the client
    client = carla.Cleint(args.host,args.port)
    try:
        world = client.get_world()                                          #grab the world
        speed_signs = world.get_actors()                                    #grab the actors
        for speed_sign in speed_signs.filter('traffic.speed_limit.*'):      #find all the speed signs
            print(speed_sign.get_location())

        actor = world.get_actor(args.carid)
    except rospy.ROSInterruptException:
        pass
