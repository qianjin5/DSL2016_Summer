#!/usr/bin/env python2

"""
quadsim.py

Written By: Adrian Esser and David Wu

Changes:
- Aug 2015 - Separated and vectorized quadrotor dynamics, PEP8

This ROS node represents a physically accurate drone object. It receives
flight commands from the ArDrone bridge just as a real drone would do, and it
publishes it's state data to the Vicon bridge too. Onboard control and
dynamics are all calculated in this file.

SUBSCRIBED TOPICS
/cmd_vel
/ardrone/takeoff
/ardrone/land
/dnn_coordinates

PUBLISHED TOPICS
/vicon/ARDroneShare/ARDroneShare
/ardrone/navdata
/gazebo/set_model_state
"""

from __future__ import division, print_function

import rospy
import numpy as np

from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped, Twist
from dsl__utilities__msg.msg import StateData
from gazebo_msgs.msg import ModelState
from ardrone_autonomy.msg import Navdata
from dsl__projects__dnn.srv import SetState
from tf import transformations

from dsl__utilities__ardrone import DroneStatus
import dsl__projects__dnn

__all__ = ['']


class PhysDrone:
    """
    Uses the drone dynamics from above to create a ROS simulation.

    Parameters
    ----------
    external_forces: list
        A list that constains all the external forces that apply to the
        quadrotor. See the forces creators below.
    """

    def __init__(self, external_forces=None):

        self.latest_cmd = Twist()
        self.fake_navdata_msg = Navdata()
        self.gazebo_msg_sim = ModelState()
        self.gazebo_msg_des = ModelState()
        self.gazebo_msg_ref = ModelState()

        # Reset the drone state to landed
        self.fake_navdata_msg.state = DroneStatus.Landed

        # Retrieve the model name
        model_param = rospy.search_param('model')
        if model_param:
            self.quad_model = rospy.get_param(model_param)
        else:
            raise EnvironmentError('No model parameter specified.')

        # by default use same name for tag as for quad_model but append '_tag'
        self.tag_model_des = self.quad_model + '_tag_des'
        self.tag_model_ref = self.quad_model + '_tag_ref'

        # Update the gazebo simulation with the model name
        self.gazebo_msg_sim.model_name = self.quad_model
        self.gazebo_msg_des.model_name = self.tag_model_des
        self.gazebo_msg_ref.model_name = self.tag_model_ref

        pos = np.zeros(3)
        pos[:3] = (rospy.get_param('~init_x', 0.),
                   rospy.get_param('~init_y', 0.),
                   rospy.get_param('~init_z', 0.))

        # default to no external forces
        if external_forces is None:
            external_forces = []

        self.quadrotor = dsl__projects__dnn.QuadrotorDynamics(
                pos,
                external_forces=external_forces)

        # Publishers
        self.pub_vicon_coords = rospy.Publisher('/vicon/{0}/{0}'.format(self.quad_model), TransformStamped, queue_size=30)
        self.pub_navdata = rospy.Publisher('ardrone/navdata', Navdata, queue_size=30)
        self.pub_gazebo = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=30)
        self.srv_set_state = rospy.Service('set_state', SetState, self.set_state)

        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel_delayed', Twist, self.update_cmd_vel_command)
        #self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.update_cmd_vel_command)
        self.sub_takeoff = rospy.Subscriber('ardrone/takeoff', Empty, self.receive_takeoff)
        self.sub_land = rospy.Subscriber('ardrone/land', Empty, self.receive_land)
        self.sub_gazebo_tag_des = rospy.Subscriber('desired_coordinates', StateData, self.update_gazebo_tag_des)
        self.sub_gazebo_tag_ref = rospy.Subscriber('path_coordinates', StateData, self.update_gazebo_tag_ref)
        #self.sub_curr_state = rospy.Subscriber('estimated_state', StateVector, self.disp_cur_state)

        # Set the /use_sim_time paramter to false for writing to bag files.
        rospy.set_param("/use_sim_time", False)

        # Set up a rospy Timer that is responsible for updating the position of
        # the drone. This timer basically runs the inner loop
        self.pub_pose = rospy.Timer(
            rospy.Duration(self.quadrotor.params.inner_loop_cycle * 1e-6),
            self.update_position)

        # Retreive important parameters from the ROS Parameter Server
        # [rad]
        self.euler_angle_max = rospy.get_param(
            self.quad_model + "_driver/euler_angle_max", 0.32)

        # Convert to m/s
        self.control_vz_max = rospy.get_param(
            self.quad_model + "_driver/control_vz_max", 2000.0) / 1000.0

        # [rad/s]
        self.control_yaw = rospy.get_param(
            self.quad_model + "_driver/control_yaw", 3.14)

    def set_state(self, msg):
        """Set the quadrotor state and return True"""
        self.latest_cmd = Twist()
        self.quadrotor.state.R = self.quadrotor.state.rpy_to_R(msg.state.euler)
        self.quadrotor.state.pos = np.asarray(msg.state.pos)
        self.quadrotor.state.vel = np.asarray(msg.state.vel)
        self.quadrotor.state.acc = np.asarray(msg.state.acc)
        self.quadrotor.state.omega = np.asarray(msg.state.omega_b)
        self.latest_cmd = Twist()
        return True

    def update_position(self, _):
        """Update dynamics based on input and publish information to ROS."""
        # Here we will set the commanded velocity in the Z direction, which
        # will depend on if the drone is taking off or hovering.
        self.alter_cmd_vel_command()

        # Check if the drone is flying, taking off, or landing. If so, we must
        # proceed to run the onboard controller and evaluate dynamics in order
        # to update the position service.
        if (self.fake_navdata_msg.state == DroneStatus.Flying or
                self.fake_navdata_msg.state == DroneStatus.TakingOff or
                self.fake_navdata_msg.state == DroneStatus.Landing):

            self.quadrotor.update_position(self.latest_cmd.linear.x,
                                           self.latest_cmd.linear.y,
                                           self.latest_cmd.linear.z,
                                           self.latest_cmd.angular.z)

        # Determine the roll, pitch, and yaw, and convert this into a
        # quaternion to be sent to Vicon
        quat = self.quadrotor.state.quaternion
        pos = self.quadrotor.state.pos.copy()

        fake_vicon_msg = TransformStamped()

        # # Measurement noise
        # pos[:2] += np.array([0.000056, 0.000090]) * np.random.randn(2)
        # pos[2] += np.random.uniform(-0.0005, 0.0005)
        #
        # rpy = self.quadrotor.state.rpy
        # rpy[:2] += np.array([0.000087, 0.00032]) * np.random.randn(2)
        # rpy[2] += np.random.uniform(-0.0005, 0.0005)
        # quat = transformations.quaternion_from_euler(*rpy)

        # Construct and send the Vicon position and rotation information
        (fake_vicon_msg.transform.translation.x,
         fake_vicon_msg.transform.translation.y,
         fake_vicon_msg.transform.translation.z) = pos

        (fake_vicon_msg.transform.rotation.x,
         fake_vicon_msg.transform.rotation.y,
         fake_vicon_msg.transform.rotation.z,
         fake_vicon_msg.transform.rotation.w) = quat

        fake_vicon_msg.header.stamp = rospy.Time.now()
        self.pub_vicon_coords.publish(fake_vicon_msg)

        # Finally update the Gazebo model
        # TODO: Add a flag perhaps to control whether this function is called
        # TODO: (does the user want a simulation?)
        self.update_gazebo_sim(quat)

        # Update navdata information
        self.update_navdata()

    def update_cmd_vel_command(self, cmd_vel):
        """Receive normalized drone command and convert to physical units."""

        # Pitch
        self.latest_cmd.linear.x = np.clip(cmd_vel.linear.x,
                                           -self.euler_angle_max,
                                           self.euler_angle_max)

        # Roll (-roll -> + y_dd)
        self.latest_cmd.linear.y = np.clip(-cmd_vel.linear.y,
                                           -self.euler_angle_max,
                                           self.euler_angle_max)

        # Z Velocity
        self.latest_cmd.linear.z = np.clip(cmd_vel.linear.z,
                                           -self.control_vz_max,
                                           self.control_vz_max)
        self.latest_cmd.angular.x = 0.0
        self.latest_cmd.angular.y = 0.0

        # Yaw Rate
        self.latest_cmd.angular.z = cmd_vel.angular.z * self.control_yaw

    def alter_cmd_vel_command(self):
        """Change cmd_vel during take off or landing to ensure stability."""
        if self.fake_navdata_msg.state == DroneStatus.TakingOff:
            self.latest_cmd.linear.x = 0.0
            self.latest_cmd.linear.y = 0.0
            self.latest_cmd.linear.z = self.quadrotor.params.takeoff_speed
            self.latest_cmd.angular.z = 0.0
        if self.fake_navdata_msg.state == DroneStatus.Landing:
            self.latest_cmd.linear.x = 0.0
            self.latest_cmd.linear.y = 0.0
            self.latest_cmd.linear.z = -self.quadrotor.params.takeoff_speed
            self.latest_cmd.angular.z = 0.0

    def receive_takeoff(self, _):
        """Set the drone status to flying and the z-pos to takeoff height."""
        if self.fake_navdata_msg.state != DroneStatus.Flying:
            print('Taking off')

            # Change the DroneStatus flag in NavData. This way when we are
            # reading in the commands from the DSL Controller, we can manually
            # set the z velocity until a specified takeoff height has been
            # reached. Then we can switch the state to flying!
            self.fake_navdata_msg.state = DroneStatus.TakingOff

            # Keep the drone from dipping under the ground at the beginning of
            # takeoff
            self.quadrotor.state.vel[2] = 0.0
            self.quadrotor.state.acc[2] = 0.0

    def receive_land(self, _):
        """Land the drone at the current position (velocities are set to 0)."""
        if self.fake_navdata_msg.state != DroneStatus.Landed:
            print('Landing')

            # Change the DroneStatus to Landing
            self.fake_navdata_msg.state = DroneStatus.Landing

            # Keep the drone from drifting during the landing process.
            self.quadrotor.state.vel[:2] = [0,0]
            self.quadrotor.state.pos[:2] = [0,0]

    def update_navdata(self):
        """Update the navdata information."""
        self.fake_navdata_msg.header.stamp = rospy.get_rostime()
        self.fake_navdata_msg.batteryPercent = 100

        # If the drone is taking off and it's position in the z direction has
        # exceeded 1.0 meters
        # then we must tell NavData that the drone is now flying
        if (self.fake_navdata_msg.state == DroneStatus.TakingOff and
                self.quadrotor.state.pos[2] > 0):
            # Rest commanded velocities
            self.latest_cmd = Twist()
            self.fake_navdata_msg.state = DroneStatus.Flying

        # If the drone is landing and it's position in the z direction has gone
        # below 0.0 meters then we must tell NavData that the drone has landed.
        # Also set z, roll, pitch, and yaw, to zero for safety.
        if (self.fake_navdata_msg.state == DroneStatus.Landing and
                self.quadrotor.state.pos[2] <= 0):
            self.fake_navdata_msg.state = DroneStatus.Landed
            self.quadrotor.state.R = np.eye(3)
            self.quadrotor.state.pos[2] = 0

        self.pub_navdata.publish(self.fake_navdata_msg)

    def update_gazebo_sim(self, quaternion):
        """Update the gazebo model with the current quadrotor state."""
        (self.gazebo_msg_sim.pose.position.x,
         self.gazebo_msg_sim.pose.position.y,
         self.gazebo_msg_sim.pose.position.z) = self.quadrotor.state.pos

        (self.gazebo_msg_sim.pose.orientation.x,
         self.gazebo_msg_sim.pose.orientation.y,
         self.gazebo_msg_sim.pose.orientation.z,
         self.gazebo_msg_sim.pose.orientation.w) = quaternion

        self.pub_gazebo.publish(self.gazebo_msg_sim)

    def update_gazebo_tag_des(self, desired_path):
        """Update the desired gazebo tag."""
        self.gazebo_msg_des.pose.position = desired_path
        self.pub_gazebo.publish(self.gazebo_msg_des)
        
    def update_gazebo_tag_ref(self, reference_path):
        """Update the reference gazebo tag."""
        self.gazebo_msg_ref.pose.position = reference_path
        self.pub_gazebo.publish(self.gazebo_msg_ref)	
    
    
if __name__ == '__main__':
    rospy.init_node('quadsim')

    # Example forces that could be added to the Drone
    wind = dsl__projects__dnn.wind_creator(np.array((-1, 0, 0)), 60)
    random = \
        dsl__projects__dnn.random_disturbance_creator(0.05 * np.eye(3))
    external_forces = [random]
    # request_node = PhysDrone(external_forces)
    request_node = PhysDrone()

    rospy.spin()
