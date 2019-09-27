"""! does things

@package carla_plugin
"""
#!/usr/bin/env python

import argparse
import signal
import sys
import time
import numpy as np
import carla
from osu_modules.dynamics.av import AV

def loop(car):
    """ main execution loop:
            initiats AV object
            run_step updates AV
    """

    #matlab vehicle scenerio test intial conditions
    #works on Town01
    transform = carla.Transform()
    transform.location.x = 302.82
    transform.location.y = 57.5001
    transform.location.z = 0
    transform.rotation.pitch = 0
    transform.rotation.yaw = -0.0958
    transform.rotation.roll = 0

    car._vehicle.set_transform(transform)

    #remove tire friction to allow for smooth angular velocity control
    vehicle_dynamics = car._vehicle.get_physics_control()
    wheel_phyics = vehicle_dynamics.wheels
    for i in range(4):
        wheel_phyics[i].tire_friction = 0.0

    vehicle_dynamics.wheels = wheel_phyics
    car._vehicle.apply_physics_control(vehicle_dynamics)
    while True:

        #path planning updates, map based AV
        car.run_step()
        time.sleep(0.001)

def exit_handler(signum, frame):
    """ main program decunstructor """

    print '\ncleaning up and closing carla plugin'
    sys.exit(1)

def test_velocity_process(car):
    """! function used for testing
        @param car: link to carla vehicle
        @return nothing
    """
    #path planning updates, map based AV
    car.run_step()

    #max_speed = car._set_speed
    #print("current location: (%s, %s, %s)"%(car._current_transform_c.location.x, \
    #    car._current_transform_c.location.y,car._current_transform_c.location.z))
    #print("next location   : (%s, %s, %s)"%(car._next_transform_c.location.x, \
    #    car._next_transform_c.location.y,car._next_transform_c.location.z))

    #current vehicle rotation matrix
    _rc = car._current_rotation_r

    #find velocity direction
    velocity_dir = _rc[:, 0]
    velocity_dir = velocity_dir.reshape(1, 3)
    velocity_dir = np.transpose(velocity_dir)

    #update velocity direction and magnitude
    #to be handled by vehicle_control Process
    car.vc.velocity_set_args['direction_x'] = velocity_dir[0, 0]
    car.vc.velocity_set_args['direction_y'] = velocity_dir[1, 0]
    car.vc.velocity_set_args['direction_z'] = velocity_dir[2, 0]

    car.vc.velocity_set_args['x_angular_vel'] = temp[0]
    car.vc.velocity_set_args['y_angular_vel'] = temp[1]
    car.vc.velocity_set_args['z_angular_vel'] = temp[2]

    car.vc.velocity_set_args['magnitude'] = input("Enter new Speed [km/hr]: ")

    #path planning algorithm and scalling function
    #solve for coefficents
    # #s(t) = a0 + a1t + a2t +a3t : s(0) = 0 s(car._sampling_time) = 1

    #definiton of path using scaling funciton: X(s) = Tcexp(log(Tc^(-1)Tn)s)
    #run the path
    time.sleep(0.1)

if __name__ == '__main__':
    #create argument parser
    ARGPARSER = argparse.ArgumentParser(description=__doc__)
    ARGPARSER.add_argument(
        '--rolename',
        metavar='rolename',
        default='hero',
        help='enter id of vehicle to control'
    )
    ARGPARSER.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)'
    )
    ARGPARSER.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)'
    )
    ARGS = ARGPARSER.parse_args()

    #connect to the client
    CLIENT = carla.Client(ARGS.host, ARGS.port)

    #world = CLIENT.load_world('Town01')
    WORLD = CLIENT.get_world()          #grab the world
    ACTORS = WORLD.get_actors()         #grab all actors

    for temp in ACTORS:
        if temp.attributes.has_key('role_name'):
            role = temp.attributes['role_name']
            if role == 'ego-vehicle':
                actor = temp
                break
            else:
                print 'actor does not exist'
                exit()

    CAR = AV(WORLD, actor)
    signal.signal(signal.SIGINT, exit_handler)
    loop(CAR)
