#!/usr/bin/env python2

# PURPOSE
# This controller takes in current state information through the /current_coordinates topic and requests waypoint information from the /path_follower node for the desired state. Based on the difference between these two states, the controller computes outputs and sends commands to the drone.
# This controller also has a keypress handler to allow for manual control of the vehicle.

# SUBSCRIBED TOPICS
# /current_coordinates
# /path_coordinates
# /ardrone/navdata

# PUBLISHED TOPICS
# /cmd_vel_ideal [rad]
# /ardrone/land 
# /ardrone/takeoff
# /ardrone/reset
# /waypoint_request


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import rospy

import time
import sys
import math
import numpy as np

# Load the DroneVideoDisplay class, which handles video display
from dsl__utilities__ardrone import DroneStatus
from dsl__utilities__ardrone import DroneVideoDisplay
# The GUI libraries
from PySide import QtCore, QtGui


###################
# IMPORT MESSAGES #
###################

from dsl__utilities__msg.msg import StateVector
from dsl__utilities__msg.msg import StateData
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty  
from std_msgs.msg import Bool

from rospy.numpy_msg import numpy_msg

from dsl__projects__dnn import Parameters

# For shutting down the QT application window in CTRL-C
import signal
from PyQt4.QtCore import QTimer
from PyQt4.QtGui import QApplication, QMessageBox
def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    QApplication.quit()


##################       Key Mapping Object     ################################
class KeyMapping(object):
  ForwardLeft      		= QtCore.Qt.Key.Key_W
  Forward          		= QtCore.Qt.Key.Key_E
  ForwardRight     		= QtCore.Qt.Key.Key_R
  Right            		= QtCore.Qt.Key.Key_F
  BackwardRight    		= QtCore.Qt.Key.Key_V
  Backward         		= QtCore.Qt.Key.Key_C
  BackwardLeft     		= QtCore.Qt.Key.Key_X
  Left             		= QtCore.Qt.Key.Key_S
  YawLeft          		= QtCore.Qt.Key.Key_A
  YawRight         		= QtCore.Qt.Key.Key_G
  IncreaseAltitude 		= QtCore.Qt.Key.Key_Q
  DecreaseAltitude 		= QtCore.Qt.Key.Key_Z
  Takeoff          		= QtCore.Qt.Key.Key_Y
  Land             		= QtCore.Qt.Key.Key_H
  Emergency        		= QtCore.Qt.Key.Key_Space
  StartHover       		= QtCore.Qt.Key.Key_I
  EndHover         		= QtCore.Qt.Key.Key_K
  Function1        		= QtCore.Qt.Key.Key_O # Custom functions
  Function2        		= QtCore.Qt.Key.Key_P #
  Function3_global 		= QtCore.Qt.Key.Key_N # global custom function 
  LandAll          		= QtCore.Qt.Key.Key_J
  TakeoffAll       		= QtCore.Qt.Key.Key_U
  DNN_Toggle       		= QtCore.Qt.Key.Key_D
  LoadTrajectory   		= QtCore.Qt.Key.Key_L
  LoadTrajectoryFromServer   	= QtCore.Qt.Key.Key_M

#####################   Useful Structures for the Controller ###################


class State:
  x      = np.array([0,0,0])
  x_dot  = np.array([0,0,0])
  x_ddot = np.array([0,0,0])

  rpy    = np.array([0,0,0],dtype=np.float64)

  def __init__(self, x_     =np.array([0,0,0]),
                     x_dot_ =np.array([0,0,0]),
                     x_ddot_=np.array([0,0,0]),
                     rpy_   =np.array([0,0,0])):
    self.x      = x_
    self.x_dot  = x_dot_
    self.x_ddot = x_ddot_
    self.rpy    = rpy_

  @classmethod
  def fromState(cls, ss):
    new_class = cls(ss.x, ss.x_dot, ss.x_ddot, ss.rpy)
    return new_class

class DroneCommand:
  roll    = 0
  pitch   = 0
  yaw_dot = 0
  z_dot   = 0

  twist = Twist()

class Status:
  drone_state       = -1 # hover/flight etc
  keyboard_override = 0
  hover             = 0
  request_waypoint  = Bool(True)
  t_last_cmd        = 0


#####################   Main Controller Code    ################################

# class DroneController(DroneVideoDisplay):
# 
#  
#   def __init__(self):
#     super(DroneController,self).__init__)

class DroneController(DroneVideoDisplay):

  # Member Variables
  current_state = State()
  desired_state = State()
  command       = DroneCommand()
  status        = Status()

  # some parameters
  tau_x = 1.5
  tau_y = 1.5
  tau_z = 0.8
  tau_w = 0.7
  zeta  = 0.707

  max_euler = 0.
  max_vz    = 0.
  max_yaw   = 0.
  g = 9.81
  
  #*****************************************************************************

  # Constructor
  def __init__(self):
    super(DroneController,self).__init__()
    rospy.loginfo('Initializing Non-linear Controller---------------------------')
    ''' add to drone_video_display
    self.takeoff = QtGui.QCommandLinkButton('Take Off', self)
    self.land = QtGui.QCommandLinkButton('Land', self)
    self.DNN = QtGui.QCommandLinkButton('DNN Toggle', self)

        
    self.takeoff.clicked.connect(self.SendTakeoff)
    self.takeoff.move(10,10)
    self.land.clicked.connect(self.SendLand)
    self.land.move(10,40)
    self.DNN.clicked.connect(self.SendToggle)
    self.DNN.move(10,70)
    '''
    self.params = Parameters()
    # Subscribers 
    # path_coordinates = desired coordinates
    # current_coordinates = vicon coordinates
    # Publishers
    
    self.test = False

    self.pubLand    = rospy.Publisher('ardrone/land', Empty, queue_size=0)
    self.pubTakeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=0)
    self.pubReset   = rospy.Publisher('ardrone/reset', Empty, queue_size=0)
    self.pubCommand = rospy.Publisher('cmd_vel_ideal', Twist, queue_size=30)
    self.pubF1      = rospy.Publisher('function1', Empty, queue_size=0)
    self.pubF2      = rospy.Publisher('function2', Empty, queue_size=0)
    self.pubF3      = rospy.Publisher('/function3', Empty, queue_size=0)
    self.pubLandAll = rospy.Publisher('/land_all', Empty, queue_size=0)
    self.pubTakeoffAll = rospy.Publisher('/takeoff_all', Empty, queue_size=0)
    self.pubDNN_Toggle = rospy.Publisher('DNN_Toggle', Empty, queue_size=0)
    self.pubLoadTraj   = rospy.Publisher('load_traj', Empty, queue_size=0)
    self.pubLoadTrajFromS   = rospy.Publisher('load_traj_fs', Empty, queue_size=0)
    

    self.sub_cur     = rospy.Subscriber('estimated_state', StateVector, self.updateCurrentState)
    self.sub_navdata = rospy.Subscriber('ardrone/navdata', Navdata, self.updateNavdata)
    self.sub_des     = rospy.Subscriber('path_coordinates', StateData, self.updateDesiredState)
    self.sub_land_all = rospy.Subscriber('/land_all', Empty, self.SendLand)    
    self.sub_takeoff_all = rospy.Subscriber('/takeoff_all', Empty, self.SendTakeoff)
    self.sub_test = rospy.Subscriber('test', Empty, self.setTest)

    # Control Parameters
    self.max_euler = rospy.get_param('ardrone_driver/euler_angle_max', 0.26)  # rads
    self.max_vz    = rospy.get_param('ardrone_driver/control_vz_max', 1.0)  # m/sec
    self.max_yaw   = rospy.get_param('ardrone_driver/control_yaw', 1.75)  # rads/s
    cmd_rate       = rospy.get_param('cmd_rate', 70);         # command rate (Hz)
    self.COMMAND_PERIOD = 1.0/cmd_rate

    # Design Parameters
    print "Getting parameters" 
    self.tau_x = rospy.get_param("~tau_x", 0.7)
    self.tau_y = rospy.get_param("~tau_y", 0.7)
    self.tau_z = rospy.get_param("~tau_z", 0.7)
    self.tau_w = rospy.get_param("~tau_w", 1.5)
    self.zeta  = rospy.get_param("~zeta",0.707);

    print 'tau_x: ', self.tau_x, '\n', 'tau_y: ', self.tau_y, '\n', 'tau_z: ', self.tau_z, '\n', 'tau_w: ', self.tau_w, '\n', 'zeta: ', self.zeta, '\n'
    print 'euler_angle_max: ', self.max_euler, '\n', 'control_vz_max: ', self.max_vz, '\n', 'control_yaw: ', self.max_yaw, '\n' 

    # Requests path waypoint
    self.pub_request  = rospy.Publisher('waypoint_request', Bool, queue_size=0)

    # Establish a timer to request waypoints at a given frequency
    self.waypointTimer  = rospy.Timer(rospy.Duration(self.COMMAND_PERIOD), self.requestWaypoint)
    self.commandTimer   = rospy.Timer(rospy.Duration(self.COMMAND_PERIOD), self.publishCommands)
    # Signal Handling in case ROS is shut down (video display closed or <Ctrl-C>)
    rospy.on_shutdown(self.hook)
    rospy.loginfo('Nonlinear controller ready-----------------------------------')

  #*****************************************************************************

  def setTest(self,msg):
    self.test = not self.test
    if self.test == True:
      print '---Test Start---'
    else:
      print '---Test Stop---'
      

  # signal handeling in case of shutdown
  def hook(self):
    self.pubLand.publish(Empty())
    print "Landing!"

  #*****************************************************************************

  def publishCommands(self, event):
    self.SendCommand(roll=self.command.roll, pitch=self.command.pitch,
                     yaw_rate=self.command.yaw_dot, z_dot=self.command.z_dot)
    return

  #*****************************************************************************

  def requestWaypoint(self,event):
    self.pub_request.publish(self.status.request_waypoint)

  #*****************************************************************************
  ##############################################
  def DSLcontroller(self, cur_state, des_state):
    import math
    import numpy as np
    # This function takes in the current state, desired state and set
    # of parameters, and applies the DSL controller equations to determine
    # the commands that would be sent to the drone over the cmd_vel topic.
    # state: (x, y, z, x', y', z', R11, ... , R33, p, q, r, z'')
    # cmd_vel: (roll_cmd, pitch_cmd, yaw_rate_cmd, z_dot_cmd)

    # z_dot_cmd = (1/tau_z)*(z_des - z_cur)
    z_dot_cmd = (2.0*self.zeta/self.tau_z)*(des_state.x_dot[2]-cur_state.x_dot[2]) + (1.0/(self.tau_z**2))*(des_state.x[2]-cur_state.x[2]) 
    # x_ddot_cmd = (2*eta/tau_x)*(x_dot_des - x_dot_cur) + (1/tau_x^2)*(x_des - x_cur)
    x_ddot_cmd = ((2.0*self.zeta/self.tau_x)*(des_state.x_dot[0]-cur_state.x_dot[0]) + (1.0/(self.tau_y**2))*(des_state.x[0]-cur_state.x[0]))
    # y_ddot_cmd = (2*eta/tau_y)*(y_dot_des - y_dot_cur) + (1/tau_y^2)*(y_des - y_cur)
    y_ddot_cmd = ((2.0 *self.zeta/self.tau_x)*(des_state.x_dot[1]-cur_state.x_dot[1]) + (1.0/(self.tau_y**2))*(des_state.x[1]-cur_state.x[1]))

    thrust = (self.g + cur_state.x_ddot[2])/(math.cos(cur_state.rpy[0]) * math.cos(cur_state.rpy[1]))
    # Determine "roll" and "pitch" in the global frame
    roll_global_cmd = math.asin(self.clamp(y_ddot_cmd/thrust,1,-1))
    pitch_global_cmd = math.asin(self.clamp(x_ddot_cmd/(thrust*math.cos(roll_global_cmd)),1,-1))
    
    pitch_cmd = pitch_global_cmd*math.cos(cur_state.rpy[2]) + roll_global_cmd*math.sin(cur_state.rpy[2])
    roll_cmd = - pitch_global_cmd*math.sin(cur_state.rpy[2]) + roll_global_cmd*math.cos(cur_state.rpy[2])

    # Determine the commanded yaw rate
    yaw_rate_cmd = (1/self.tau_w)*(des_state.rpy[2]-cur_state.rpy[2])

    # Create output vector
    # The minus sign before roll_cmd is due to the sign convention roll angles
    # (Positive Roll causes acceleration in negative y-direction)
    cmd_vel = np.array([-roll_cmd, pitch_cmd, yaw_rate_cmd, z_dot_cmd])
    #print cmd_vel
    return cmd_vel
  #################################
  def determineCommands(self): 
    
    # Save variables so they are not over-written in mid-calculation
    des  = State.fromState(self.desired_state)
    curr = State.fromState(self.current_state)
    
    # Z-velocity command m/sec)
    z_velocity_out =  ((2.0*self.zeta/self.tau_z) * (des.x_dot[2] - curr.x_dot[2]) + (1.0/(self.tau_z**2))*(des.x[2] - curr.x[2]) )

    # Yaw rate command (rad/sec)??
    yaw_err = np.mod(des.rpy[2]-curr.rpy[2] + np.pi, 2.*np.pi) - np.pi
    yaw_velocity_out = (1.0 / self.tau_w) * yaw_err

    # Roll/Pitch Commands
    # determine the mass-normalized thrust
    # thrust = np.linalg.norm(np.array([0.,0.,self.g]) + curr.x_ddot)
    # calculate the desired acceleration in x and y (global coordinates, [m/s^2] )
    ax = (2.0*self.zeta/self.tau_x)*(des.x_dot[0]-curr.x_dot[0]) + (1.0/(self.tau_x*self.tau_x))*(des.x[0]-curr.x[0])
    ay = (2.0*self.zeta/self.tau_x)*(des.x_dot[1]-curr.x_dot[1]) + (1.0/(self.tau_x*self.tau_x))*(des.x[1]-curr.x[1])
    thrust = np.linalg.norm(np.array([ax, ay, curr.x_ddot[2] + self.g]))

    # keep ax,ay < thrust (so that arcsin is defined)
    if thrust == 0.0:
        ax_clamped = 1.0
        ay_clamped = 1.0
    else:
        ax_clamped = self.clamp(ax / thrust, 1.0)
        ay_clamped = self.clamp(ay / thrust, 1.0)
 
    # Rotate desired accelerations into drone's body frame
    ax_b =  ax_clamped*np.cos(curr.rpy[2]) + ay_clamped*np.sin(curr.rpy[2])
    ay_b = -ax_clamped*np.sin(curr.rpy[2]) + ay_clamped*np.cos(curr.rpy[2])

    # convert acceleration into roll/pitch angles [rad]
    pitch_out =  np.arcsin(ax_b)
    roll_out  = -np.arcsin(ay_b)



    # send the commands to the drone if the keyboard is not currently being used
    # if(self.status.keyboard_override == 0):
    #   self.status.t_last_cmd = time.time()
    #   self.SendCommand(roll_out, pitch_out, yaw_velocity_out, z_velocity_out)

    # Store the latest set of commands
    self.command.pitch   = pitch_out
    self.command.roll    = roll_out
    self.command.yaw_dot = yaw_velocity_out
    self.command.z_dot   = z_velocity_out
    
    self.command.twist.angular.x = (des.x[0]-curr.x[0])
    self.command.twist.angular.y = (des.x_dot[0]-curr.x_dot[0])
    
    # matlab ver
    #cmd_vel = self.DSLcontroller(curr, des)
    #self.command.pitch = cmd_vel[1]
    #self.command.roll    = cmd_vel[0]
    #self.command.yaw_dot = cmd_vel[2]
    #self.command.z_dot   = cmd_vel[3]
  #****************************************************************************
 
  # Publish Commands to the drone if we are not in hover mode.
  def SendCommand(self, roll, pitch, yaw_rate, z_dot):
    if self.test == False:  
      self.command.twist.linear.x = pitch
      self.command.twist.linear.y = roll
      self.command.twist.linear.z = z_dot
      self.command.twist.angular.z = yaw_rate
    else:
      self.command.twist.linear.x = 0
      self.command.twist.linear.y = 0.1
      self.command.twist.linear.z = 0
      self.command.twist.angular.z = 0

    # make sure the drone is not taking off
    if (self.status.drone_state != DroneStatus.TakingOff): 
      self.pubCommand.publish(self.command.twist)


  #***************    Callback Functions for Measurements   *******************

  def updateNavdata(self,nav_msg):
    self.status.drone_state = nav_msg.state

  def viconNoise(self, state):
    # Vicon noise
    state.pos += np.random.normal(0, self.params.vicon_noise_pos, 3)
    state.vel += np.random.normal(0, self.params.vicon_noise_vel, 3)
    state.euler += np.random.normal(0, self.params.vicon_noise_omega, 3)
    return state
  
  #****************************************************************************
  # This method updates the current state of the drone
  
  def updateCurrentState(self,curr_data):

    # Vicon noise
    curr_data = self.viconNoise(curr_data)
    
    # update the state information
    self.current_state.x      = curr_data.pos
    self.current_state.x_dot  = curr_data.vel
    self.current_state.x_ddot = curr_data.acc
    self.current_state.rpy    = curr_data.euler

    # Determine the commands to be sent to the drone
    self.determineCommands()

  #****************************************************************************

  # Update the desired state 
  def updateDesiredState(self,desiredState):

    # Update the desired state information
    self.desired_state.x      = np.array([desiredState.x,  desiredState.y,  desiredState.z],  dtype=np.float64)
    self.desired_state.x_dot  = np.array([desiredState.vx, desiredState.vy, desiredState.vz], dtype=np.float64)
    self.desired_state.x_ddot = np.array([desiredState.ax, desiredState.ay, desiredState.az], dtype=np.float64)
    self.desired_state.rpy    = np.array([desiredState.roll, desiredState.pitch, desiredState.yaw], dtype=np.float64)

  #****************************************************************************

  def clamp(self, num, upper=1.0, lower=None):
    if (lower is None):
      num = max(min(num,upper),-1.0*upper)
    else:
      num = max(min(num,upper),lower)
    return (num)

  #****************************************************************************

  # Sends a land signal to all flying drones
  def SendLandAll(self):
    self.pubLandAll.publish(Empty())
  
  # Sends a takeoff signal to all flying drones
  def SendTakeoffAll(self):
    self.pubTakeoffAll.publish(Empty())

  # Send an emergency (or reset) message to the ardrone driver
  def SendEmergency(self):
    self.pubReset.publish(Empty())

  # Send a takeoff message to the ardrone driver
  def SendTakeoff(self,called=None):
    self.pubTakeoff.publish(Empty())
  
  # Send a landing message to the ardrone driver
  def SendLand(self,called=None):
    self.pubLand.publish(Empty())

  def SendToggle(self,called=None):
    self.pubDNN_Toggle.publish(Empty())
    
  def LoadTrajectory(self,called=None):
    self.pubLoadTraj.publish(Empty()) 
  
  def LoadTrajectoryFromServer(self, called = None):
    self.pubLoadTrajFromS.publish(Empty())
  
  # Send Empty messages on function1, function2 and /function3 topics
  def sendFunction1(self):
    self.pubF1.publish(Empty())

  def sendFunction2(self):
    self.pubF2.publish(Empty())

  def sendFunction3(self):
    self.pubF3.publish(Empty())

  #****************************************************************************
  # Keyboard Controller
  #****************************************************************************

  # This method is called when a key is pressed. It overrides the automated commands.
  def keyPressEvent(self, event):
    key = event.key()

    # If the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat()):
      # Turn on override
      self.status.keyboard_override = 1 # turn on override
      roll_out = 0.0
      pitch_out = 0.0
      yaw_velocity_out = 0.0
      z_velocity_out = 0.0

      # Handle the important cases first!
      if key == KeyMapping.LandAll:
        self.SendLandAll()
      elif key == KeyMapping.TakeoffAll:
        self.SendTakeoffAll()
      elif key == KeyMapping.Emergency:
        self.SendEmergency()
      elif key == KeyMapping.Takeoff:
        self.SendTakeoff()
      elif key == KeyMapping.Land:
        self.SendLand()
      elif key == KeyMapping.StartHover:
        self.hover = 1
      elif key == KeyMapping.EndHover:
        self.hover = 0
      elif key == KeyMapping.Function1: # send empty messages on /function1 topic 
        self.sendFunction1() 
      elif key == KeyMapping.Function2: # send empty messages on /function2 topic 
        self.sendFunction2() 
      elif key == KeyMapping.Function3_global: # send empty messages on /function2 topic 
        self.sendFunction3() 
      elif key == KeyMapping.DNN_Toggle: # send empty messages on /function2 topic 
        self.SendToggle()
      elif key == KeyMapping.LoadTrajectory: # send empty messages on /function2 topic 
        self.LoadTrajectory()
      elif key == KeyMapping.LoadTrajectoryFromServer: # send empty messages on /function2 topic 
        self.LoadTrajectoryFromServer()
      else:
        # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
        if key == KeyMapping.YawLeft:
          yaw_velocity_out = self.max_yaw
        elif key == KeyMapping.YawRight:
          yaw_velocity_out = -self.max_yaw
        elif key == KeyMapping.ForwardLeft:
          pitch_out = self.max_euler
          roll_out = self.max_euler
        elif key == KeyMapping.Forward:
          pitch_out = self.max_euler
        elif key == KeyMapping.ForwardRight:
          pitch_out = self.max_euler
          roll_out = -self.max_euler
        elif key == KeyMapping.Right:
          roll_out = -self.max_euler
        elif key == KeyMapping.BackwardRight:
          pitch_out = -self.max_euler
          roll_out = -self.max_euler
        elif key == KeyMapping.Backward:
          pitch_out = -self.max_euler
        elif key == KeyMapping.BackwardLeft:
          pitch_out = -self.max_euler
          roll_out = self.max_euler
        elif key == KeyMapping.Left:
          roll_out = self.max_euler
        elif key == KeyMapping.IncreaseAltitude:
          z_velocity_out = self.max_vz
        elif key == KeyMapping.DecreaseAltitude:
          z_velocity_out = -self.max_vz
      
      self.SendCommand(roll_out, pitch_out, yaw_velocity_out, z_velocity_out)

  #****************************************************************************

  def keyReleaseEvent(self,event):
    key = event.key()

    # If the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat()):
      # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
      self.status.keyboard_override = 0 # turn off override
      self.SendCommand(0, 0, 0, 0)

#####################     Main Code to Run      ################################
if __name__=='__main__':

  # set up the signal handeler 
  signal.signal(signal.SIGINT, sigint_handler)

  # First we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('nonlinear_controller')
  
  # Now we construct our Qt Application and associated windows
  app = QtGui.QApplication(sys.argv)
  display = DroneController()
  display.show()
  # executes the QT application
  status = app.exec_()

  # and only progresses to here once the application has been shutdown
  rospy.signal_shutdown('Great Flying!')
  sys.exit(status)

