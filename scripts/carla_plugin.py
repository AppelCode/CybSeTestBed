#!/usr/bin/env python

# ==============================================================================
# -- Setup ---------------------------------------------------------------------
# ==============================================================================
import glob
import os
import sys
import time 
import signal

#uncomment and adjust if carla is not in your pyhton path
#try:
#    sys.path.append(glob.glob('/home/matt/Documents/CybSe/CARLA/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
#        sys.version_info.major,
#        sys.version_info.minor,
#        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
#except IndexError:
#    pass

import rospy
from carla_matlab_dynamics_ros_plugin.msg import PathPlanner
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

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
from threading import Lock

# ==============================================================================
# -- Robotics Stuff ------------------------------------------------------------
# ==============================================================================

class RoboticHelper():
    
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

    #TODO: update function
    def to_transform(self,carlaTransform):
        rotation = self.to_rotation(carlaTransform.rotation)
        location = carlaTransform.location

        x = location.x
        y = location.y
        z = location.z

        transform = np.array([])
        
        return transform
    
    def to_rotation(self,carlaRotation):
        roll = math.radians(carlaRotation.roll)
        pitch = math.radians(carlaRotation.pitch)
        yaw = math.radians(carlaRotation.yaw)

        if (roll == 360) or (roll ==-360):
            roll = 0
        if (pitch == 360) or (pitch == -360):
            pitch = 0
        if (yaw == 360) or (yaw == -360):
            yaw = 0

        yaw_matrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]])

        pitch_matrix = np.array([[math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]])

        roll_matrix = np.array([[1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]])

        return np.matmul(yaw_matrix, pitch_matrix, roll_matrix)

        #return np.array([[C(a)*C(b), C(a)*S(b)*S(g)-S(a)*C(g), C(a)*S(b)*C(g)+S(a)*S(g)],
        #                 [S(a)*C(b), S(a)*S(b)*S(g)+C(a)*C(g), S(a)*S(b)*C(g)-C(a)*S(g)],
        #                 [-S(b), C(b)*S(g), C(b)*C(g)]])

    def convert_orientation(self,rotation_matrix,array):
        return np.matmul(rotation_matrix,array)

    #TODO: implement function
    def rot_to_quaternion(self,SO3):
        return 1, 2

# ==============================================================================
# -- Vehicle Contorl Process ---------------------------------------------------
# ==============================================================================

class VehicleVelocityControl():
    #TODO: add proper intialization for orientation 
    def __init__(self,id):
        self.car_id = id 
        self.original_sigint = signal.getsignal(signal.SIGINT)
        self.manager = multiprocessing.Manager()
        self.velocity_set_args = self.manager.dict()    
        self.velocity_set_args['direction_x'] = 0
        self.velocity_set_args['direction_y'] = 0
        self.velocity_set_args['direction_z'] = 0
        self.velocity_set_args['magnitude'] = 0         #only used for testing
        self.velocity_set_args['x_angular_vel'] = 0
        self.velocity_set_args['y_angular_vel'] = 0
        self.velocity_set_args['z_angular_vel'] = 0
        self.mutex = Lock()

        #start parallel process
        numjobs = 1 
        jobs = []
        for i in range(numjobs):
            p = multiprocessing.Process(target=self.vehicle_control)
            jobs.append(p)
            p.start()

    #process used to move vehicle in carla
    def vehicle_control(self):

        def exit_child_process(signum, frame):
            signal.signal(signal.SIGINT, self.original_sigint)
            exit(1)
        
        client = carla.Client(args.host,args.port)
        world = client.get_world()                  #grab the world
        actors = world.get_actors()                 #grab all actors
        car = actors.find(self.car_id)                   #find a specific car
        
        #process exit request
        signal.signal(signal.SIGINT, exit_child_process)

        #run vehicle in x direction
        while True:

            self.mutex.acquire()
            try:  
                magnitude = self.velocity_set_args['magnitude']
                x_dir = self.velocity_set_args['direction_x']
                y_dir = self.velocity_set_args['direction_y']
                z_dir = self.velocity_set_args['direction_z']

                #used for testing ros communication 
                #TODO: remove magnitude used in testing
                x_dir = magnitude*x_dir
                y_dir = magnitude*y_dir
                z_dir = magnitude*z_dir

                x_ang = float(180/math.pi)*self.velocity_set_args['x_angular_vel']
                y_ang = float(180/math.pi)*self.velocity_set_args['y_angular_vel']
                z_ang = float(180/math.pi)*self.velocity_set_args['z_angular_vel']

            finally:
                self.mutex.release()

                run_velocity_dir = carla.Vector3D(x=x_dir,y=y_dir,z=z_dir)
                run_ang_velocity = carla.Vector3D(x=x_ang,y=y_ang,z=z_ang)
                car.set_velocity(run_velocity_dir)
                #TODO: add angular velocity input when lat control is finished
                #car.set_angular_velocity(run_ang_velocity)
                time.sleep(0.1)
        
    def _update_vehicle_velocity(self,twist):

        self.velocity_set_args['direction_x'] = np.asscalar(twist[0])
        self.velocity_set_args['direction_y'] = np.asscalar(twist[1])
        self.velocity_set_args['direction_z'] = np.asscalar(twist[2])
        self.velocity_set_args['x_angular_vel'] = np.asscalar(twist[3])
        self.velocity_set_args['y_angular_vel'] = np.asscalar(twist[4])
        self.velocity_set_args['z_angular_vel'] = np.asscalar(twist[5])
        
# ==============================================================================
# -- Autonomous Vehicle --------------------------------------------------------
# ==============================================================================
#using a gloabl variable for the vehicle will not create link and call descunstrutor 
#at exit

class AV(RoamingAgent):
    # Robotic Helper is used for Screw Theory Mathmatics, northwestern modern robotics 
    # source cited in imports
    rh = RoboticHelper()

    # World orientation is left hand cooridinate 
    world_orientation = np.array([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, -1]])
    
    def __init__(self,world,vehicle_local):
        #super(AV,self).__init__(vehicle_local)
        #vehicle enviroenment setup information
        self._vehicle = vehicle_local
        self._world = world
        self._map = self._world.get_map()
        
        #vehicle specific information
        self.role_name = "ego_vehicle"
        self._set_speed = None

        #vehicle velocity (updated through ros, from matlab model)
        self._twist = np.zeros((6,1))
        
        #vehicle control process class
        #self.vc = VehicleVelocityControl(self._vehicle.id)

        #waypoint generator
        self._local_planner = LocalPlanner(self._vehicle)
        self._theta = 0


        #Path information initialization
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform_c =self._current_waypoint.transform
        self._current_transform_r = self.rh.to_transform(self._current_transform_c)
        self._current_rotation_c =self._current_transform_c.rotation
        self._current_rotation_r = self.rh.to_rotation(self._current_transform_c.rotation)

        self._next_waypoint = self._current_waypoint.next(1)[0]
        self._next_transform_c = self._next_waypoint.transform
        self._next_transform_r = self.rh.to_transform(self._next_transform_c)
        self._next_rotation_c = self._next_transform_c.rotation
        self._next_rotation_r = self.rh.to_rotation(self._next_rotation_c)

        #Ros definitions
        # matalb model velocity to carla
        self.control_vel_subscriber = rospy.Subscriber(
            "/carla_plugin/" + self.role_name + "/vehicle_model_velocity",
            Twist, self._update_vehicle_velocity)

        # matlab model steering to carla
        self.control__steering_subscriber = rospy.Subscriber(
            "/carla_plugin/" + self.role_name + "/vehicle_model_steering",
            Float64, self._update_vehicle_steering)
        
        # test ros connection
        self.test_subscriber = rospy.Subscriber(
            "/carla_plugin/" + self.role_name + "/vehicle_model_test",
            Float64, self._test_ros)

        # to send ackermann set values to matlab
        self.carla_set_publisher = rospy.Publisher(
            "/carla_plugin/" + self.role_name + "/vehicle_set_values",
            AckermannDrive, queue_size=1)

        # to send path plan to matlab
        self.carla_path_publisher = rospy.Publisher(
            "/carla_plugin/" + self.role_name + "/vehicle_path",
            PathPlanner, queue_size=1)

        rospy.init_node(self.role_name, anonymous=True)

    #used to udate current vehicle orientation and location
    #vehicle orientaion is based on vehicle not watpoint
    def _update_current_position(self):
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform_c =self._vehicle.get_transform()
        self._current_transform_r = self.rh.to_transform(self._current_transform_c)
        self._current_rotation_c =self._current_transform_c.rotation
        self._current_rotation_r = self.rh.to_rotation(self._current_transform_c.rotation)

    #use ackermann drive message to update set points
    def _update_set_points(self):

        #update speed limit
        self._set_speed = self._vehicle.get_speed_limit()
        msg = AckermannDrive()

        #set steering angle and steering change limit (0 means as fast as possible)
        msg.steering_angle = self._theta
        msg.steering_angle_velocity = 0
    
        #set desired speed, acceleration and jerk (0 means as fast as possible)
        msg.speed = 7#float(1/3.6)*self._set_speed
        msg.acceleration = 0
        msg.jerk = 0

        #publish set points
        self.carla_set_publisher.publish(msg)

    #desired angle to next point
    def _update_path_angle(self):

        self._next_waypoint = self._current_waypoint.next(1)[0]
        self._next_transform_c = self._next_waypoint.transform
        self._next_transform_r = self.rh.to_transform(self._next_transform_c)
        self._next_rotation_c = self._next_transform_c.rotation
        self._next_rotation_r = self.rh.to_rotation(self._next_rotation_c)

        #TODO: go from rotation notation to roll pitch yaw
        #TODO: push this calculation to robotic helper
        #logrithm of rotation
        R = np.matmul(self._next_rotation_r,np.transpose(self._current_rotation_r))
        trace = np.trace(R)
        omega_skew = np.zeros((3,3))

        if np.array_equal(R,np.eye(3,3)):
            self._theta = 0

        elif trace == -1:
            self._theta = math.pi
            if R[2,2] != -1:
                omega_skew = 1/math.sqrt(2(1+R[2,2]))*np.transpose(np.array(R[0,2], R[1,2],1+R[2,2]))
            elif R[1,1] != -1:
                omega_skew = 1/math.sqrt(2(1+R[1,1]))*np.transpose(np.array(R[0,1], 1+ R[1,1],R[2,1]))
            elif R[0,0] != -1:
                omega_skew = 1/math.sqrt(2(1+R[0,0]))*np.transpose(np.array(1+R[0,0], R[1,0],R[2,0]))

        else:
            self._theta = math.acos((0.5)*(trace-1))
            if self._theta > 0:
                omega_skew = (1/(2*math.sin(self._theta))*np.subtract(R,np.transpose(R))) 

        omg = mr.so3ToVec(omega_skew)       

        if omg[2] > 0:
            self._theta = -self._theta

    #updates current position and
    #updates target waypoints
    #TODO: finish implementation
    def _update_path_waypoints(self):

        path = []
        pose = PoseStamped()
        for i in range(100):
            self._next_waypoint = self._current_waypoint.next(0.1)[0]
            self._next_transform_c = self._next_waypoint.transform
            self._next_transform_r = self.rh.to_transform(self._next_transform_c)

            pose.pose.position.x = self._current_transform_c.location.x
            pose.pose.position.y = self._current_transform_c.location.y
            pose.pose.position.z = self._current_transform_c.location.z

            self._next_rotation_c = self._next_transform_c.rotation
            self._next_rotation_r = self.rh.to_rotation(self._next_rotation_c)

            q = np.zeros((4,1))
            theta, omega_hat = self.rh.rot_to_quaternion(self._next_rotation_r)

            q[0,0] = math.cos(theta/2)
            q[1:4] = omega_hat*math.sin(theta/2)

            pose.pose.orientation.w = q[0,0]
            pose.pose.orientation.x = q[1,0]
            pose.pose.orientation.y = q[2,0]
            pose.pose.orientation.z = q[3,0]

            path.append(pose)

    #ROS callback used to test
    def _test_ros(self,mag):
        self.vc.velocity_set_args['magnitude'] = mag.data

    #ROS callback used to update linear and angular velocity from matlab
    def _update_vehicle_velocity(self, vehicle_model_velocity):
        #TODO: adjust orientation
        linear_vel = vehicle_model_velocity.linear
        temp_linear = np.zeros((3,1))
        temp_linear[0,0] = linear_vel.x
        temp_linear[1,0] = linear_vel.y
        temp_linear[2,0] = linear_vel.z
        self._twist[0:3] = temp_linear

        #representation of vehicle frame in relation to the world rotated to match cars orientation
        #v = Rc*Rw*v
        self._twist[0:3] = self.rh.convert_orientation( \
            self._current_rotation_r,self.rh.convert_orientation(self.world_orientation,self._twist[0:3]))
        

        angular_vel = vehicle_model_velocity.angular
        temp_angular = np.zeros((3,1))
        temp_angular[0,0] = angular_vel.x
        temp_angular[1,0] = angular_vel.y
        temp_angular[2,0] = angular_vel.z
        self._twist[3:6] = temp_angular

        #representation of vehicle frame in relation to the world rotated to match cars orientation
        #vc = Rc*Rw*vm
        self._twist[3:6] = self.rh.convert_orientation( \
            self._current_rotation_r,self.rh.convert_orientation(self.world_orientation,self._twist[3:6]))

        self.vc.mutex.acquire()
        try:

            #TODO: remove magnitude after testing is validated
            self.vc.velocity_set_args['magnitude'] = 1
            self.vc._update_vehicle_velocity(self._twist)
        finally:
            self.vc.mutex.release()

    #ROS callback used to update steering angle
    def _update_vehicle_steering(self,angle):
        self.steering_angle = angle 
        
    def run_step(self):
        self._update_current_position()        
        self._update_path_angle()
        self._update_set_points()
        #control = self._local_planner.run_step()
        #self._vehicle.apply_control(control)

    #def __del__(self):
        #print("deleted")

# ==============================================================================
# -- main loop -----------------------------------------------------------------
# ==============================================================================
def loop(car):

    car._vehicle.set_simulate_physics(True)

    while True:

        #path planning updates, map based AV 
        car.run_step()
        
        #max_speed = car._set_speed
        #print("current location: (%s, %s, %s)"%(car._current_transform_c.location.x, \
        #    car._current_transform_c.location.y,car._current_transform_c.location.z))
        #print("next location   : (%s, %s, %s)"%(car._next_transform_c.location.x, \
        #    car._next_transform_c.location.y,car._next_transform_c.location.z))

        #current vehicle rotation matrix
        #Rc = car._current_rotation_r

        #find velocity direction
        #velocity_dir = Rc[:,0]  
        #velocity_dir = velocity_dir.reshape(1,3)     
        #velocity_dir = np.transpose(velocity_dir)

        #update velocity direction and magnitude 
        #to be handled by vehicle_control Process
        #car.vc.velocity_set_args['direction_x'] = velocity_dir[0,0]
        #car.vc.velocity_set_args['direction_y'] = velocity_dir[1,0]
        #car.vc.velocity_set_args['direction_z'] = velocity_dir[2,0]

        #car.vc.velocity_set_args['x_angular_vel'] = temp[0]
        #car.vc.velocity_set_args['y_angular_vel'] = temp[1]
        #car.vc.velocity_set_args['z_angular_vel'] = temp[2]

        #car.vc.velocity_set_args['magnitude'] = input("Enter new Speed [km/hr]: ") 

        #path planning algorithm and scalling function
        #solve for coefficents
        # #s(t) = a0 + a1t + a2t +a3t : s(0) = 0 s(car._sampling_time) = 1

        #definiton of path using scaling funciton: X(s) = Tcexp(log(Tc^(-1)Tn)s)
        #run the path
        time.sleep(0.1)

def exit_handler(signum, frame):
    print('\ncleaning up and closing carla plugin')
    sys.exit(1)

if __name__ == '__main__':
    #create argument parser
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--rolename',
        metavar = 'rolename',
        default = 'hero',
        help = 'enter id of vehicle to control'
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
                if role == "ego_vehicle":
                    actor = temp

        car = AV(world,actor)
        signal.signal(signal.SIGINT, exit_handler)
        loop(car)

    except rospy.ROSInterruptException:
        pass