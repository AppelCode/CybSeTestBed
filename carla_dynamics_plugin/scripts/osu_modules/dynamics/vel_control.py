#!/usr/bin/env python
""" control velocity vector of single carla vehicle """

import multiprocessing
from threading import Lock
import numpy as np
import carla

class VehicleVelocityControl(object):
    """ controls linear and angular velocity of carla vehicle
            note: spawns new process to run the control
    """
    def __init__(self, carid, world):

        self._car_id = carid
        self._manager = multiprocessing.Manager()
        self._world = world

        self.velocity_set_args = self._manager.dict()
        self.velocity_set_args['direction_x'] = 0
        self.velocity_set_args['direction_y'] = 0
        self.velocity_set_args['direction_z'] = 0
        self.velocity_set_args['magnitude'] = 0         #only used for testing
        self.velocity_set_args['x_angular_vel'] = 0
        self.velocity_set_args['y_angular_vel'] = 0
        self.velocity_set_args['z_angular_vel'] = 0
        self.mutex = Lock()

        #start parallel process for vehicle velocity control
        numjobs = 1
        jobs = []
        for i in range(numjobs):
            _p = multiprocessing.Process(target=self._vehicle_control)
            jobs.append(_p)
            _p.start()

    #process used to move vehicle in carla
    def _vehicle_control(self):

        CLIENT = carla.Client('127.0.0.1', 2000)
        WORLD = CLIENT.get_world()          #grab the world
        ACTORS = WORLD.get_actors()         #grab all actors
        car = ACTORS.find(self._car_id)

        #run vehicle in x direction
        while True:

            self.mutex.acquire()
            try:
                magnitude = self.velocity_set_args['magnitude']
                x_dir = self.velocity_set_args['direction_x']
                y_dir = self.velocity_set_args['direction_y']
                z_dir = self.velocity_set_args['direction_z']

                #used for testing ros communication
                x_dir = magnitude*x_dir
                y_dir = magnitude*y_dir
                z_dir = magnitude*z_dir

                x_ang = self.velocity_set_args['x_angular_vel']
                y_ang = self.velocity_set_args['y_angular_vel']
                z_ang = self.velocity_set_args['z_angular_vel']

            finally:
                self.mutex.release()

                run_velocity_dir = carla.Vector3D(x=x_dir, y=y_dir, z=z_dir)
                run_ang_velocity = carla.Vector3D(x=x_ang, y=y_ang, z=z_ang)
                car.set_velocity(run_velocity_dir)
                car.set_angular_velocity(run_ang_velocity)

    def _update_vehicle_velocity(self, twist):

        self.velocity_set_args['direction_x'] = np.asscalar(twist[0])
        self.velocity_set_args['direction_y'] = np.asscalar(twist[1])
        self.velocity_set_args['direction_z'] = np.asscalar(twist[2])
        self.velocity_set_args['x_angular_vel'] = np.asscalar(twist[3])
        self.velocity_set_args['y_angular_vel'] = np.asscalar(twist[4])
        self.velocity_set_args['z_angular_vel'] = np.asscalar(twist[5])
