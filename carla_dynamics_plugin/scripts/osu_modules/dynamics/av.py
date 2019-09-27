""" @package AV

autonmous vehicle structure
"""
#!/usr/bin/env python

import numpy as np

import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header


from osu_modules.dynamics.vel_control import VehicleVelocityControl
from ..nav.roamer import Roamer
from ..tools.robotic_helper import RoboticHelper

######## TESTING ########
WAYPOINTS = np.loadtxt('refPoses.dat')
WAYPOINTS = np.asarray(WAYPOINTS)

WAYPOINTSACT = np.loadtxt('refPoses_act.dat')
WAYPOINTSACT = np.asarray(WAYPOINTSACT)

class AV(object):
    """ Autonomous Vehicle class used to operate carla vehicle

        super Roamer determines path
        uses VehicleVelocityControl to operate vehicle
    """

    # Robotic Helper is used for Screw Theory Mathmatics, northwestern modern robotics
    # source cited in imports
    rh = RoboticHelper()

    # World orientation is left hand cooridinate
    world_orientation = np.array([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, -1]])

    def __init__(self, world, vehicle_local):
        #super(AV, self).__init__(vehicle_local)

        #vehicle enviroenment setup information
        self._vehicle = vehicle_local
        self._world = world
        self._map = self._world.get_map()

        #vehicle specific information
        self.role_name = "ego_vehicle"
        self._set_speed = None
        self._steering_angle = 0

        #vehicle velocity (updated through ros, from matlab model)
        self._twist = np.zeros((6, 1))

        #vehicle control process class
        self._vc = VehicleVelocityControl(self._vehicle.id, self._world)

        #waypoint generator
        #super(AV, self).run_step()

        #Path information initialization
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform_c = self._current_waypoint.transform
        self._current_transform_r = self.rh.to_transform(self._current_transform_c)
        self._current_rotation_c = self._current_transform_c.rotation
        self._current_rotation_r = self.rh.to_rotation(self._current_transform_c.rotation)

        #Ros definitions
        # matalb model velocity to carla
        self.control_vel_subscriber = rospy.Subscriber(
            "/carla_plugin/" + self.role_name + "/vehicle_model_velocity",
            Twist, self._update_vehicle_velocity)

        # matlab model steering to carla
        self.control_steering_subscriber = rospy.Subscriber(
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
            Path, queue_size=1)

        # to send path plan to matlab
        self.carla_path_publisher_act = rospy.Publisher(
            "/carla_plugin/" + self.role_name + "/vehicle_path_act",
            Path, queue_size=1)


        rospy.init_node(self.role_name, anonymous=True)


    #=============================================================================
    # Run step functions
    #=============================================================================

    #used to udate current vehicle orientation and location
    #vehicle orientaion is based on vehicle not watpoint
    def _update_current_position(self):
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._current_transform_c = self._vehicle.get_transform()
        self._current_transform_r = self.rh.to_transform(self._current_transform_c)
        self._current_rotation_c = self._current_transform_c.rotation
        self._current_rotation_r = self.rh.to_rotation(self._current_transform_c.rotation)

    #use ackermann drive message to update set points
    def _update_set_points(self):

        #update speed limit
        self._set_speed = self._vehicle.get_speed_limit()
        msg = AckermannDrive()

        #set steering angle and steering change limit (0 means as fast as possible)
        msg.steering_angle = 0
        msg.steering_angle_velocity = 0

        #set desired speed, acceleration and jerk (0 means as fast as possible)
        #TODO: adjust back to speed limit
        msg.speed = 5#float(1/3.6)*self._set_speed
        msg.acceleration = 0
        msg.jerk = 0

        #publish set points
        self.carla_set_publisher.publish(msg)

    #desired angle to next point
    #TODO: unused
    def _update_path_angle(self):

        next_waypoint = self._current_waypoint.next(1)[0]
        next_transform_c = next_waypoint.transform
        next_transform_r = self.rh.to_transform(next_transform_c)
        next_rotation_c = next_transform_c.rotation
        next_rotation_r = self.rh.to_rotation(next_rotation_c)

        #TODO: go from rotation notation to roll pitch yaw

        _rotation = np.matmul(next_rotation_r, np.transpose(self._current_rotation_r))
        omg, theta = self.rh.log_rot(_rotation)

    #updates current position and
    #updates target waypoints
    #TODO: finish implementation use roamer
    def _update_path_waypoints(self):

        hazard, target_waypoint = super(AV,self).roamer_run_step()
        #create current waypoint pose
        posecurrent = Pose()
        q = self.rh.rot_to_quaternion(self._current_rotation_r)

        posecurrent.position.x = self._current_transform_c.location.x
        posecurrent.position.y = self._current_transform_c.location.y
        posecurrent.position.z = self._current_transform_c.location.z
        posecurrent.orientation.w = q[0, 0]
        posecurrent.orientation.x = q[1, 0]
        posecurrent.orientation.y = q[2, 0]
        posecurrent.orientation.z = q[3, 0]

        path = []
        posetarget = PoseStamped()

        target_location = target_waypoint.location
        target_rotation_r = self.rh.to_rotation(target_waypoint.transform.rotation)
        q = self.rh.rot_to_quaternion(target_rotation_r)

        posetarget.pose.position.x = target_location.x
        posetarget.pose.position.y = target_location.y
        posetarget.pose.position.z = target_location.z
        posetarget.pose.orientation.w = q[0, 0]
        posetarget.pose.orientation.x = q[1, 0]
        posetarget.pose.orientation.y = q[2, 0]
        posetarget.pose.orientation.z = q[3, 0]

        path.append(posetarget)

        #ROS send path
        #msg = PathPlanner()
        #msg.current = posecurrent
        #msg.target = path

        self.carla_path_publisher.publish(msg)

    def _update_path_test(self):
        _n = np.asarray(WAYPOINTS.shape)

        i = 0

        path = Path()

        while i < _n[0]:

            point = PoseStamped()
            header = Header()
            header.seq = i
            header.frame_id = 'map'
            header.stamp = rospy.Time.now()

            point.header = header
            point.pose.position.x = WAYPOINTS[i, 0]
            point.pose.position.y = WAYPOINTS[i, 1]
            point.pose.position.z = 0

            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = 1
            point.pose.orientation.w = WAYPOINTS[i, 2]

            path.poses.append(point)
            i = i + 1

        #create current waypoint pose
        posecurrent = Pose()
        q = self.rh.rot_to_quaternion(self._current_rotation_r)

        posecurrent.position.x = self._current_transform_c.location.x
        posecurrent.position.y = self._current_transform_c.location.y
        posecurrent.position.z = self._current_transform_c.location.z
        posecurrent.orientation.w = q[0, 0]
        posecurrent.orientation.x = q[1, 0]
        posecurrent.orientation.y = q[2, 0]
        posecurrent.orientation.z = q[3, 0]

        #msg = PathPlanner()
        #msg.current = posecurrent
        #msg.target = path

        header.seq = 0
        header.frame_id = 'map'
        header.stamp = rospy.Time.now()
        path.header = header

        self.carla_path_publisher.publish(path)

        #act ref_poses
        n = np.asarray(WAYPOINTSACT.shape)

        i = 0

        path = Path()

        while i < n[0]:

            point = PoseStamped()
            header = Header()
            header.seq = i
            header.frame_id = 'map'
            header.stamp = rospy.Time.now()

            point.header = header
            point.pose.position.x = WAYPOINTSACT[i, 0]
            point.pose.position.y = WAYPOINTSACT[i, 1]
            point.pose.position.z = 0

            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = 1
            point.pose.orientation.w = WAYPOINTSACT[i, 2]

            path.poses.append(point)
            i = i + 1

        #create current waypoint pose
        posecurrent = Pose()
        q = self.rh.rot_to_quaternion(self._current_rotation_r)

        posecurrent.position.x = self._current_transform_c.location.x
        posecurrent.position.y = self._current_transform_c.location.y
        posecurrent.position.z = self._current_transform_c.location.z
        posecurrent.orientation.w = q[0, 0]
        posecurrent.orientation.x = q[1, 0]
        posecurrent.orientation.y = q[2, 0]
        posecurrent.orientation.z = q[3, 0]

        #msg = PathPlanner()
        #msg.current = posecurrent
        #msg.target = path

        header.seq = 0
        header.frame_id = 'map'
        header.stamp = rospy.Time.now()
        path.header = header

        self.carla_path_publisher_act.publish(path)

    #=============================================================================
    # ROS calbacks
    #=============================================================================
    #ROS callback used to update linear and angular velocity from matlab
    def _update_vehicle_velocity(self, vehicle_model_velocity):
        #TODO: adjust orientation
        linear_vel = vehicle_model_velocity.linear
        temp_linear = np.zeros((3, 1))
        temp_linear[0, 0] = linear_vel.x
        temp_linear[1, 0] = linear_vel.y
        temp_linear[2, 0] = linear_vel.z
        self._twist[0:3] = temp_linear

        #representation of vehicle frame in relation to the world rotated to match cars orientation
        #v = Rc*Rw*v
        self._twist[0:3] = self.rh.convert_orientation( \
            self._current_rotation_r, self.rh.convert_orientation(self.world_orientation,\
            self._twist[0:3]))


        angular_vel = vehicle_model_velocity.angular
        temp_angular = np.zeros((3, 1))
        temp_angular[0, 0] = angular_vel.x
        temp_angular[1, 0] = angular_vel.y
        temp_angular[2, 0] = angular_vel.z
        self._twist[3:6] = temp_angular

        #representation of vehicle frame in relation to the world rotated to match cars orientation
        #vc = Rc*Rw*vm
        self._twist[3:6] = self.rh.convert_orientation(\
            self._current_rotation_r, self.rh.convert_orientation(self.world_orientation,\
            self._twist[3:6]))

        self._vc.mutex.acquire()
        try:

            #TODO: remove magnitude after testing is validated
            self._vc.velocity_set_args['magnitude'] = 1
            self._vc._update_vehicle_velocity(self._twist)
        finally:
            self._vc.mutex.release()

    #ROS callback used to update steering angle
    def _update_vehicle_steering(self, angle):
        self._steering_angle = angle

    #ROS callback used to testpose
    def _test_ros(self, mag):
        self._vc.velocity_set_args['magnitude'] = mag.data

       #=============================================================================
    # Step execution
    #=============================================================================

    def run_step(self):
        self._update_current_position()
        self._update_path_test()
        self._update_set_points()

        #self._update_path_angle()
        #self._update_path_waypoints()
        #control = self._local_planner.run_step()
        #self._vehicle.apply_control(control)

    #def __del__(self):
        #print("deleted")
