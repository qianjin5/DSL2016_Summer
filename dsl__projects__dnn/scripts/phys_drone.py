#!/usr/bin/env python2

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import math
import random
import tf
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState


from ardrone_autonomy.msg import Navdata
from drone_status import DroneStatus


class PhysDrone:
    freq = 1
    following_path = True
    request_msg    = Bool(True)

    x_vel = 0
    y_vel = 0
    z_vel = 0

    v_max = 2.0 # max velocity in m/s

    m_drone = 0.40 # mass of drone in kg
    x_v = 0
    y_v = 0
    z_v = 0

    roll_v = 0
    pitch_v = 0
    yaw_v = 0

    x_sim = 0
    y_sim = 0
    z_sim = 0
    roll_sim = 0
    pitch_sim = 0
    yaw_sim = 0

    yaw_incr = 0.1
    k_pos = 0.01
    spin_freq = 100


    fake_vicon_msg   = TransformStamped()
    latest_cmd       = Twist()
    fake_navdata_msg = Navdata()
    gazebo_msg       = ModelState()

    def __init__(self):
      # self.sub_path_coords    = rospy.Subscriber('/path_coordinates', StateData, self.updateCommands)
      self.sub_cmd_vel        = rospy.Subscriber('/cmd_vel', Twist, self.updateTwistCommand)
      self.sub_takeoff        = rospy.Subscriber('/ardrone/takeoff', Empty, self.recieveTakeoff)
      self.sub_land           = rospy.Subscriber('/ardrone/land', Empty, self.recieveLand)
      self.pub_vicon_coords   = rospy.Publisher('/vicon/ARDroneShare/ARDroneShare',TransformStamped)
      self.pub_navdata        = rospy.Publisher('/ardrone/navdata',Navdata)
      self.pub_gazebo         = rospy.Publisher('/gazebo/set_model_state',ModelState) # For publishing state data to Gazebo Simulation

      # simulation parameters
      max_yaw = rospy.get_param('control_yaw', 1.75) #rads/s
      self.spin_freq = rospy.get_param('sim_freq',200) # Hz
      self.v_max = math.sqrt(2.0)*rospy.get_param('v_max_sim', 2.0) # max magnitude of velocity vector
      self.pos_eps = rospy.get_param('sim_pos_eps', 0.3)/self.spin_freq #noise in m/s for position
      self.yaw_eps = rospy.get_param('sim_yaw_eps', 0.15)/self.spin_freq #noise in rad/s for position
      self.yaw_incr = max_yaw/self.spin_freq
      self.k_pos = 1.2/self.spin_freq
      self.t_land = 1.0 # seconds it takes to land
      self.takeoff_height = 1.0

      # set up transform related services
      self.pub_pose = rospy.Timer(rospy.Duration(1.0/self.spin_freq),self.updatePosition)
      self.listener = tf.TransformListener()
      self.tf_br    = tf.TransformBroadcaster()
      self.name     = '/vicon/ARDroneShare/ARDroneShare'
      self.world    = '/Frisbee'


      # for Navdata
      self.pub_pose = rospy.Timer(rospy.Duration(1.0/self.spin_freq),self.updateNavdata)

    def clamp(self, num, upper, lower=None):
      if (lower is None):
        num = max(min(num,upper),-1.0*upper)
        return (num)
      else:
        num = max(min(num,upper,lower))
        return (num)

    # publish position services the same way as vicon_bridge
    def updatePosition(self,event):

      if (self.fake_navdata_msg.state == DroneStatus.Flying):

        # latest_cmd = angle
        # update velocity instead of position
        x_vel_incr_local = 0
        y_vel_incr_local = 0

        thrust = 0
        if (self.latest_cmd.linear.x == self.latest_cmd.linear.y == self.latest_cmd.linear.z == self.latest_cmd.angular.x == self.latest_cmd.angular.y == self.latest_cmd.angular.z == 0.0):
          self.x_vel = 0.0
          self.y_vel = 0.0
          self.z_vel = 0.0
        else:
          thrust = self.m_drone*9.81/(math.cos(self.roll_sim)*math.cos(self.pitch_sim))
          # velocity incraments in drone frame
          x_vel_incr_local = thrust*math.sin(self.pitch_sim)
          y_vel_incr_local = thrust*math.sin(self.roll_sim)
          # self.x_vel += thrust*math.sin(self.pitch_sim)/self.spin_freq
          # self.y_vel += thrust*math.sin(self.roll_sim)/self.spin_freq
          self.z_vel  = self.latest_cmd.linear.z

        self.x_vel += (x_vel_incr_local*math.cos(self.yaw_sim) - y_vel_incr_local*math.sin(self.yaw_sim))/self.spin_freq + random.gauss(0,self.pos_eps) # self.k_pos*(math.cos(self.yaw_sim)*self.latest_cmd.linear.x - math.sin(self.yaw_sim)*self.latest_cmd.linear.y) + random.gauss(0, self.pos_eps) # +ve forward
        self.y_vel += (y_vel_incr_local*math.cos(self.yaw_sim) + x_vel_incr_local*math.sin(self.yaw_sim))/self.spin_freq + random.gauss(0,self.pos_eps) # self.k_pos*(math.cos(self.yaw_sim)*self.latest_cmd.linear.y + math.sin(self.yaw_sim)*self.latest_cmd.linear.x) + random.gauss(0, self.pos_eps)# +ve right

        # limit drone velocity
        self.x_vel = self.clamp(self.x_vel, self.v_max)
        self.y_vel = self.clamp(self.y_vel, self.v_max)
        self.z_vel = self.clamp(self.z_vel, self.v_max)

        # update position
        self.x_sim += self.x_vel/self.spin_freq # + random.gauss(0,self.pos_eps) # self.k_pos*(math.cos(self.yaw_sim)*self.latest_cmd.linear.x - math.sin(self.yaw_sim)*self.latest_cmd.linear.y) + random.gauss(0, self.pos_eps) # +ve forward
        self.y_sim += self.y_vel/self.spin_freq # + random.gauss(0,self.pos_eps)
        self.z_sim += self.z_vel/self.spin_freq # + random.gauss(0,self.pos_eps)
        # self.x_sim += (self.x_vel*math.cos(self.yaw_sim) - self.y_vel*math.sin(self.yaw_sim))/self.spin_freq + random.gauss(0,self.pos_eps) # self.k_pos*(math.cos(self.yaw_sim)*self.latest_cmd.linear.x - math.sin(self.yaw_sim)*self.latest_cmd.linear.y) + random.gauss(0, self.pos_eps) # +ve forward
        # self.y_sim += (self.y_vel*math.cos(self.yaw_sim) + self.x_vel*math.sin(self.yaw_sim))/self.spin_freq + random.gauss(0,self.pos_eps) # self.k_pos*(math.cos(self.yaw_sim)*self.latest_cmd.linear.y + math.sin(self.yaw_sim)*self.latest_cmd.linear.x) + random.gauss(0, self.pos_eps)# +ve right
        # self.z_sim += self.z_vel/self.spin_freq # self.k_pos*self.latest_cmd.linear.z + random.gauss(0, self.pos_eps) # +ve up

        # update yaw
        # convert a number from -1 to 1 to angular rate in rad/s
        # +ve right handed
        self.yaw_sim = (self.yaw_sim + self.yaw_incr*self.latest_cmd.angular.z)%(2.0*math.pi) + random.gauss(0, self.yaw_eps)
    
      quat = tf.transformations.quaternion_from_euler(0,0,self.yaw_sim)
      # self.tf_br.sendTransform((self.x_sim, self.y_sim, self.z_sim), quat, rospy.Time.now(), self.name, self.world)

      self.fake_vicon_msg.transform.translation.x = 0.0 #self.x_sim
      self.fake_vicon_msg.transform.translation.y = 0.0 #self.y_sim
      self.fake_vicon_msg.transform.translation.z = self.z_sim

      self.fake_vicon_msg.transform.rotation.x = quat[0]
      self.fake_vicon_msg.transform.rotation.y = quat[1]
      self.fake_vicon_msg.transform.rotation.z = quat[2]
      self.fake_vicon_msg.transform.rotation.w = quat[3]

      self.fake_vicon_msg.header.stamp = rospy.get_rostime()
      self.pub_vicon_coords.publish(self.fake_vicon_msg)

      #print "I just published a vicon coordinate!"

      # Here I'm going to add in a message published to Gazebo which will be used to simulate the drone flying

      self.gazebo_msg.model_name = "quadrotor"

      self.gazebo_msg.pose.position.x = 0.0
      self.gazebo_msg.pose.position.y = 0.0
      self.gazebo_msg.pose.position.z = self.z_sim

      self.gazebo_msg.pose.orientation.x = quat[0]
      self.gazebo_msg.pose.orientation.y = quat[1]
      self.gazebo_msg.pose.orientation.z = quat[2]
      self.gazebo_msg.pose.orientation.w = quat[3]

      self.gazebo_msg.twist.linear.x  = 0.0
      self.gazebo_msg.twist.linear.y  = 0.0
      self.gazebo_msg.twist.linear.z  = 0.0

      self.gazebo_msg.twist.angular.x = 0.0
      self.gazebo_msg.twist.angular.y = 0.0
      self.gazebo_msg.twist.angular.z = 0.0

      self.pub_gazebo.publish(self.gazebo_msg)

    def updateNavdata(self,event):
      self.fake_navdata_msg.header.stamp = rospy.get_rostime()
      self.fake_navdata_msg.batteryPercent = 100
      self.pub_navdata.publish(self.fake_navdata_msg)

    # store the latest cmd from cmd_vel
    def updateTwistCommand(self, twist_cmd):

      self.latest_cmd.linear.x = twist_cmd.linear.x
      self.latest_cmd.linear.y = twist_cmd.linear.y
      self.latest_cmd.linear.z = twist_cmd.linear.z

      self.pitch_sim = twist_cmd.linear.x
      self.roll_sim  = twist_cmd.linear.y

      self.latest_cmd.angular.x = twist_cmd.angular.x
      self.latest_cmd.angular.y = twist_cmd.angular.y
      self.latest_cmd.angular.z = twist_cmd.angular.z

    def recieveTakeoff(self,msg):
      if (self.fake_navdata_msg.state != DroneStatus.Flying):
        print 'Taking off'
        # self.fake_navdata_msg.state = DroneStatus.TakingOff
        rospy.sleep(self.t_land)
        self.z_sim = self.takeoff_height
        self.fake_navdata_msg.state = DroneStatus.Flying

    def recieveLand(self,msg):
      if (self.fake_navdata_msg.state != DroneStatus.Landed):
        print 'Landing (phys_drone)'
        # self.fake_navdata_msg.state = DroneStatus.Landing
        rospy.sleep(self.t_land)
        self.z_sim = 0
        self.fake_navdata_msg.state = DroneStatus.Landed

# This class defines an object for the state of the drone.
# The state contains the position, velocity, and acceleration information,
# the rotation matrix (which implies pitch, roll, and yaw),
# and the angular velocity information.
class State:

    def __init__(self):

        # Position (m):
        self.x = 0
        self.y = 0
        self.x = 0

        # Velocity (m/s):
        self.x_d = 0
        self.y_d = 0
        self.z_d = 0

        # Acceleration (m/s/s):
        self.x_dd = 0
        self.y_dd = 0
        self.z_dd = 0

        # Rotation Matrix (unitless):
        self.R_11 = 0
        self.R_12 = 0
        self.R_13 = 0
        self.R_21 = 0
        self.R_22 = 0
        self.R_23 = 0
        self.R_31 = 0
        self.R_32 = 0
        self.R_33 = 0

        # Angular velocities (rad/s)
        self.p = 0
        self.q = 0
        self.r = 0

# These are the parameters defining the physics of the drone
# and the tuning parameters for all of the controllers.

# They are currently taken from:
# Schoellig, A., Hehn, M., Lupashin, S., and D'Andrea, R. (2011)
# Feasibility of Motion Primitives for Choreographed Quadrocopter Flight
# Proceedings of the American Control Conference
class Parameters:

    def __init__(self):

        test = False
        if test:
            self.m        = 0.468,   # m, mass of vehicle (kg)
            self.g        = 9.8,     # g, mass normalized gravitational force (m/s^2)
            self.L        = 0.17,    # L, vehicle arm length (m)
            self.K        = 0.016,   # K, motor constant, determined experimentally
            self.Ix       = 0.0023,  # Ix, inertia around the body's x-axis (kg-m^2)
            self.Iy       = 0.0023,  # Iy, inertia around the body's y-axis (kg-m^2)
            self.Iz       = 0.0046,  # Iz, inertia around the body's z-axis (kg-m^2)
            self.fmin     = 0.17,    # fmin, mass normalized minimum rotor force (m/s^2)
            self.fmax     = 6.0,     # fmax, mass normalized maximum rotor force (m/s^2)
            self.vmax     = 2.0,     # vmax, maximum quadrotor velocity (m/s)
            self.eta      = 0.707,   # eta, damping ratio
            self.tau_z    = 0.6,     # tau_z, time constant for vertical direction
            self.tau_Iz   = 2.5,     # tau_Iz, integral time constant for vertical direction
            self.tau_yaw  = 1.5,     # tau_yaw, time constant for yaw rate
            self.tau_Iyaw = 2.5,     # tau_Iyaw, integral time constant for yaw rate
            self.eta_y    = 0.707,   # eta_y, damping ratio
            self.tau_y    = 0.6,     # tau_y, time constant for x and y direction
            self.tau_Iu   = 2.5,     # tau_Iu, integral time constant for x and y dir.
            self.tau_p    = 0.6,     # tau_p, time constant for roll rate
            self.tau_q    = 0.6,     # tau_q, time constant for pitch rate
            self.tau_r    = 0.6,     # tau_r, time constant for yaw rate
            self.tau_rp   = 0.6,     # tau_rp, time constant
            self.tau_f    = 0.1,     # tau_f, time constant for force integration
        else:
            self.m = 1.477  # m, mass of vehicle (kg)
            self.g = 9.8  # g, mass normalized gravitational force (m/s^2)
            self.L = 0.18  # L, vehicle arm length (m)
            self.K = 0.26  # K, motor constant, determined experimentally
            self.Ix = 0.01152  # Ix, inertia around the body's x-axis (kg-m^2)
            self.Iy = 0.01152  # Iy, inertia around the body's y-axis (kg-m^2)
            self.Iz = 0.0218  # Iz, inertia around the body's z-axis (kg-m^2)
            self.fmin = 0.17  # fmin, mass normalized minimum rotor force (m/s^2)
            self.fmax = 6.0  # fmax, mass normalized maximum rotor force (m/s^2)
            self.vmax = 2.0  # vmax, maximum quadrotor velocity (m/s)
            self.eta = 0.707  # eta, damping ratio
            self.tau_z = 1.0  # tau_z, time constant for vertical direction
            self.tau_Iz = 0.05  # tau_Iz, integral time constant for vertical direction
            self.tau_yaw = 0.55  # tau_yaw, time constant for yaw rate
            self.tau_Iyaw = 0.01  # tau_Iyaw, integral time constant for yaw rate
            self.eta_y = 0.707  # eta_y, damping ratio
            self.tau_y = 1.7  # tau_y, time constant for x and y direction
            self.tau_Iu = 2.5  # tau_Iu, integral time constant for x and y dir.
            self.tau_p = 0.18  # tau_p, time constant for roll rate
            self.tau_q = 0.18  # tau_q, time constant for pitch rate
            self.tau_r = 0.1  # tau_r, time constant for yaw rate
            self.tau_rp = 0.18  # tau_rp, time constant
            self.tau_f = 0.1  # tau_f, time constant for force integration

        self.CR       = 0.6     # coefficient of restitution

if __name__ == '__main__':
    rospy.init_node('phys_drone')
    request_node = PhysDrone()

    rospy.spin()

      

