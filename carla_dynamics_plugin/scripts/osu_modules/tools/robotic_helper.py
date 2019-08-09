"""! does things

@package robotic_helper
"""
#!/usr/bin/env python

import math
import numpy as np
import modern_robotics as mr  # http://hades.mech.northwestern.edu/images/7/7f/MR.pdf [1]

class RoboticHelper(object):
    """ used for robotic screw theory needed to translate
        MATLAB dynamics into CARLAs world
    """

    #return numpy representation of SE3 transform matrix
    #
    # param carla_transform: carla object representing a transformation
    # return: numpy transform matrix (as defined in reference[1] modern_robotics import textbook)
    #
    #   [[Calpha*Cbeta Calpha*Sbeta*Sgamma-Salpha*Cgamma Calpha*Sbeta*Cgamma+SalphaS*gamma x],
    #    [Salpha*Cbeta Salpha*Sbeta*Sgamma+Calpha*Cgamma Salpha*Sbeta*Cgamma-Calpha*Sgamma y],
    #    [-Sbeta       Cbeta*Sgamma                      Cbeta*Cgamm                       z],
    #    [0            0                                 0                                 1]]
    #

    #TODO: update function
    def to_transform(self, carla_transform):
        """ returns se3 transform matrix from carla transform

            @param carla_transform
            @return transform
        """
        rotation = self.to_rotation(carla_transform.rotation)
        location = carla_transform.location

        x = location.x
        y = location.y
        z = location.z

        transform = np.array([])

        return transform

    #return numpy representation of SO3 rotation matrix
    #
    # param carla_transform: carla object representing a transformation
    # return: numpy transform matrix (as defined in reference[1] modern_robotics import textbook)
    #
    #   [[Calpha*Cbeta Calpha*Sbeta*Sgamma-Salpha*Cgamma Calpha*Sbeta*Cgamma+SalphaS*gamma],
    #    [Salpha*Cbeta Salpha*Sbeta*Sgamma+Calpha*Cgamma Salpha*Sbeta*Cgamma-Calpha*Sgamma],
    #    [-Sbeta       Cbeta*Sgamma                      Cbeta*Cgamm                      ]]
    #
    def to_rotation(self, carla_rotation):
        """ return SO3 rotation matrix from carla rotation

            @param carla_rotation
            @return rotation
        """
        roll = math.radians(carla_rotation.roll)
        pitch = math.radians(carla_rotation.pitch)
        yaw = math.radians(carla_rotation.yaw)

        if (roll == 360) or (roll == -360):
            roll = 0
        if (pitch == 360) or (pitch == -360):
            pitch = 0
        if (yaw == 360) or (yaw == -360):
            yaw = 0

        yaw_matrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],\
            [math.sin(yaw), math.cos(yaw), 0],\
            [0, 0, 1]])

        pitch_matrix = np.array([[math.cos(pitch), 0, math.sin(pitch)],\
            [0, 1, 0],\
            [-math.sin(pitch), 0, math.cos(pitch)]])

        roll_matrix = np.array([[1, 0, 0],\
            [0, math.cos(roll), -math.sin(roll)],\
            [0, math.sin(roll), math.cos(roll)]])

        return np.matmul(yaw_matrix, pitch_matrix, roll_matrix)

    #adjust frame of refrence
    #
    # param rotation_matrix: SO3 representaion of another frame of refeennce
    # param array: current frame of reference frame
    #
    # return: roation_matrix*array -> the representation of array in new frame
    #
    def convert_orientation(self, rotation_matrix, array):
        """ adjust frame of refernce

            @param so3 matrix
            @param array
            @return so3*array: i.e array representated in rotation frame so3
        """
        return np.matmul(rotation_matrix, array)

    # SO3 to unit quanternion
    def rot_to_quaternion(self, so3):
        """ convert so3 to quaternion

            @param so3
            @return so3 -> quanternion
        """
        omg, theta = self.log_rot(so3)
        _q = np.zeros((4, 1))
        _q[0, 0] = math.cos(theta/2)
        _q[1:4] = omg*math.sin(theta/2)

        return _q

    # Logrithm of Rotation Calculation
    def log_rot(self, so3):
        """ return the logarithm of rotation,
                i.e.e theta and omg that correspond to so3
        """

        trace = np.trace(so3)
        omega_skew = np.zeros((3, 3))

        if np.array_equal(so3,np.eye(3, 3)):
            theta = 0

        elif trace == -1:
            theta = math.pi
            if so3[2, 2] != -1:
                omega_skew = 1/math.sqrt(2*(1+so3[2, 2]))*np.transpose\
                (np.array(so3[0, 2], so3[1, 2], 1+so3[2, 2]))
            elif so3[1, 1] != -1:
                omega_skew = 1/math.sqrt(2*(1+so3[1, 1]))*np.transpose\
                (np.array(so3[0, 1], 1+ so3[1, 1], so3[2, 1]))
            elif so3[0, 0] != -1:
                omega_skew = 1/math.sqrt(2*(1+so3[0, 0]))*np.transpose\
                (np.array(1+so3[0, 0], so3[1, 0], so3[2, 0]))

        else:
            theta = math.acos((0.5)*(trace-1))
            if theta > 0:
                omega_skew = (1/(2*math.sin(theta))*np.subtract(so3, np.transpose(so3)))

        omg = np.zeros((3, 1))
        omg = mr.so3ToVec(omega_skew)
        omg = omg.reshape((3, 1))

        if omg[2] > 0:
            theta = -theta

        return omg, theta
