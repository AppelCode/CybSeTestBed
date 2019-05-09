#!/usr/bin/env python

######################################################################################
# Setup for carla 
import glob
import os
import sys
import time 

try:
    sys.path.append(glob.glob('/home/matt/Documents/CybSe/CARLA/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

######################################################################################
"""
Testing transform waypoints within carls using carla roaming agent to 
find waypoint path

Current Path and target path will be printed then the user will be asked
to continue:
- enter yes 

"""
import rospy

import carla
from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.roaming_agent import RoamingAgent

import modern_robotics as mr #http://hades.mech.northwestern.edu/images/7/7f/MR.pdf [1]

import math
import time
import numpy as np
import argparse
from sympy import *
import multiprocessing

# ==============================================================================
# -- Robotics ------------------------------------------------------------------
# ==============================================================================

class ScallingFunciton():

    def __init__(self):
        _a0 = 0
        _a1 = 0
        _a2 = 0
        _a3 = 0

    #solve for scalling based on current dynamics
    def _update_function(self,T,v,a):

        a0,a1,a2,a3,t = symbols('a0 a1 a2 a3 t')
        s = a0+a1*t+a2*t**2+a3*t**3     #create expression for s
        s = lambdify(t,s)  #create python function for s

        #solve for a0 using s(0) = 0
        sol = solve([Eq(s(0),0)],[a0,a1,a2,a3])
        try:
            a0 = sol[a0]
        except:
            print('no a0 in solution dict')

        s = a0+a1*t+a2*t**2+a3*t**3     #reevaluate s(t)
        s = lambdify(s,t)               #create python function

        #solve for a1 using s(T) = 1
        sol = solve([Eq(s(T),1)],[a1,a2,a3])
        try:
            a1 = sol[a1]
        except:
            print('no a0 in solution dict')

        s = a0+a1*t+a2*t**2+a3*t**3     #reevaluate s(t)
        s = lambdify(t,s)               #create python function
        sd = lambdify(t,diff(s(t),t))   #find difference equation

        #solve for a2 using sd(0)=v
        sol = solve([Eq(sd(T),v)],[a2,a3])
        try:
            a2 = sol[a2]
        except:
            print('no a0 in solution dict')

        s = a0+a1*t+a2*t**2+a3*t**3     #reevaluate s(t)
        s = lambdify(t,s)               #create python function
        sd = lambdify(t,diff(s(t),t))   #find difference equation

        #solve for a3 using sd(T)=v
        sol = solve([Eq(sd(T),v)],[a3])
        try:
            a3 = sol[a3]
        except:
            print('no a0 in solution dict')

        self._a0 = a0
        self._a1 = a1
        self._a2 = a2
        self._a3 = a3
        


class roboticHelper():
    
    def __init__(self):
        self.sf = ScallingFunciton()
    
    #return numpy representation of transform matrix
    #
    # param carlaTransform: carla object representing a transformation
    # return: numpy transform matrix (as defined in reference[1] modern_robotics import textbook)
    #
    #   [[Calpha*Cbeta Calpha*Sbeta*Sgamma-Salpha*Cgamma Calpha*Sbeta*Cgamma+SalphaS*gamma x],
    #    [Salpha*Cbeta Salpha*Sbeta*Sgamma+Calpha*Cgamma Salpha*Sbeta*Cgamma-Calpha*Sgamma y],
    #    [-Sbeta       Cbeta*Sgamma                      Cbeta*Cgamm                       z],
    #    [0            0                                 0                                 1]]
    # 
    def _to_transform(self,carlaTransform):
        rotation = carlaTransform.rotation
        location = carlaTransform.location

        g = rotation.roll
        b = rotation.pitch
        a = rotation.yaw

        x = location.x
        y = location.y
        z = location.z

        S = self.S
        C = self.C
        
        return np.array([[C(a)*C(b), C(a)*S(b)*S(g)-S(a)*C(g), C(a)*S(b)*C(g)+S(a)*S(g), x],
                         [S(a)*C(b), S(a)*S(b)*S(g)+C(a)*C(g), S(a)*S(b)*C(g)-C(a)*S(g), y],
                         [-S(b), C(b)*S(g), C(b)*C(g), z],
                         [0, 0, 0, 1]])
    
    def _to_rotation(self,carlaRotation):
        roll = math.radians(carlaRotation.roll)
        pitch = math.radians(carlaRotation.pitch)
        yaw = math.radians(carlaRotation.yaw)

        if (roll == 360) or (roll ==-360):
            roll = 0
        if (pitch == 360) or (pitch == -360):
            pitch = 0
        if (yaw == 360) or (yaw == -360):
            yaw = 0

        yaw_matrix = np.matrix([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        pitch_matrix = np.matrix([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        roll_matrix = np.matrix([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        print(roll,pitch,yaw)
        return np.matmul(yaw_matrix, pitch_matrix, roll_matrix)

        #return np.array([[C(a)*C(b), C(a)*S(b)*S(g)-S(a)*C(g), C(a)*S(b)*C(g)+S(a)*S(g)],
        #                 [S(a)*C(b), S(a)*S(b)*S(g)+C(a)*C(g), S(a)*S(b)*C(g)-C(a)*S(g)],
        #                 [-S(b), C(b)*S(g), C(b)*C(g)]])

    def S(self,num):
        return math.sin(num)

    def C(self,num):
        return math.cos(num)

    def _update_scalling_function(self,T,v,a):
        self.sf._update_function(T,v,a)

        
# ==============================================================================
# -- Autonomous Vehicle --------------------------------------------------------
# ==============================================================================
class AV(RoamingAgent):
    rh = roboticHelper()
    def __init__(self,vehicle,world):
        super(AV,self).__init__(vehicle)
        #vehicle enviroenment setup information
        self._world = world
        self._vehicle = vehicle
        self._map = self._world.get_map()

        #Path information
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform_c =self._current_waypoint.transform
        self._current_transform_r = self.rh._to_transform(self._current_waypoint.transform)
        self._current_rotation_c =self._current_transform_c.rotation
        self._current_rotation_r = self.rh._to_rotation(self._current_transform_c.rotation)

        #self._next_waypoint = self._current_waypoint.next(1)[0]
        #self._next_transform_c = self._next_waypoint.transform
        #self._next_transform_r = self.rh._to_transform(self._next_waypoint.transform)
        #self._next_rotation_c = self._next_waypoint.rotation
        #self._next_rotation_r = self.rh._to_rotation(self._next_waypoint.rotation)

        #map reference SO(3) matrix
        self._map_orientation_r = np.array([[1,0,0],
                                            [0,1,0],
                                            [0,0,1]])     
        self._map_orientation_r = np.transpose(self._map_orientation_r)

    def _update_current_position(self):
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform = self._current_waypoint.transform
        self._current_location = self._current_transform.location
        self._current_rotation = self._current_transform.rotation

    def _update_path(self):
        self._next_waypoint = self._current_waypoint.next(1)[0]
        self._next_transform = self._next_waypoint.transform

# ==============================================================================
# -- Vehicle Contorl Process ---------------------------------------------------
# ==============================================================================

#global intialization for multiprocess sharing
manager = multiprocessing.Manager()
velocity_set_args = manager.dict()    
velocity_set_args['direction_x'] = 0
velocity_set_args['direction_y'] = 0
velocity_set_args['direction_z'] = 0
velocity_set_args['magnitude'] = 1 

def vehicle_control(velocity_set_args,car_id):
    client = carla.Client(args.host,args.port)
    world = client.get_world()                  #grab the world
    actors = world.get_actors()                 #grab all actors
    car = actors.find(car_id)                   #find a specific car
    #run vehicle in x direction
    while True:
        x_dir = velocity_set_args['direction_x']
        y_dir = velocity_set_args['direction_y']
        z_dir = velocity_set_args['direction_z']
        magnitude = velocity_set_args['magnitude']
        run_velocity_dir = carla.Vector3D(x=x_dir,y=y_dir,z=z_dir)
        car.set_velocity(magnitude*run_velocity_dir)
        time.sleep(0.01)

# ==============================================================================
# -- main loop -----------------------------------------------------------------
# ==============================================================================
def loop(car):

    while True:

        #path planning updates
        #car._update_current_position()
        #car._update_path()

        #current vehicle rotation matrix
        Rc = car._current_rotation_r

        #find velocity direction
        velocity_dir = Rc[:,0]  
        velocity_dir = velocity_dir.reshape(1,3)     
        velocity_dir = np.transpose(velocity_dir)

        #update velocity direction and magnitude 
        #to be handled by vehicle_control Process
        velocity_set_args['direction_x'] = velocity_dir[0,0]
        velocity_set_args['direction_y'] = velocity_dir[1,0]
        velocity_set_args['direction_z'] = velocity_dir[2,0]
        velocity_set_args['magnitude'] = input("Enter new Speed [km/hr]: ") 
        
        #path planning algorithm and scalling function
        #solve for coefficents
        # #s(t) = a0 + a1t + a2t +a3t : s(0) = 0 s(car._sampling_time) = 1

        #definiton of path using scaling funciton: X(s) = Tcexp(log(Tc^(-1)Tn)s)
        #run the path

if __name__ == '__main__':
    #create argument parser
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--carid',
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
    client = carla.Client(args.host,args.port)
    try:
        world = client.get_world()          #grab the world
        actors = world.get_actors()         #grab all actors
        for temp in actors:
            if temp.attributes.has_key('role_name'):
                role = temp.attributes['role_name']
                if role == "hero":
                    actor = temp
        
        car = AV(actor,world)               #create AV object

        #start parallel process
        numjobs = 1 
        jobs = []
        for i in range(numjobs):
            p = multiprocessing.Process(target=vehicle_control,args=[velocity_set_args,actor.id])
            jobs.append(p)
            p.start()

        loop(car)
    except rospy.ROSInterruptException:
        pass