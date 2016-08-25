#!/usr/bin/env python2

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib

roslib.load_manifest('dsl__utilities__ardrone')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image  # for receiving the video feed
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from dsl__utilities__ardrone import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui


# Some Constants
CONNECTION_CHECK_PERIOD = 250  # ms
GUI_UPDATE_PERIOD = 20  # ms
DETECT_RADIUS = 4  # the radius of the circle drawn when a tag is detected


class DroneVideoDisplay(QtGui.QMainWindow):
    StatusMessages = {
        DroneStatus.Emergency: 'Emergency',
        DroneStatus.Inited: 'Initialized',
        DroneStatus.Landed: 'Landed',
        DroneStatus.Flying: 'Flying',
        DroneStatus.Hovering: 'Hovering',
        DroneStatus.Test: 'Test (?)',
        DroneStatus.TakingOff: 'Taking Off',
        DroneStatus.GotoHover: 'Going to Hover Mode',
        DroneStatus.Landing: 'Landing',
        DroneStatus.Looping: 'Looping (?)'
    }
    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'

    def __init__(self):
        # Construct the parent class
        super(DroneVideoDisplay, self).__init__()
        rospy.logwarn('DONT USE THIS:\n'
                      'use "from dsl__utilities__ardrone import '
                      'DroneVideoDisplay" instead.')
        # Setup our very basic GUI - a label which fills the whole window and holds our image
        self.MODEL = rospy.get_namespace()

        model_param = rospy.search_param('model')
        if model_param:
            self.MODEL = rospy.get_param(model_param)
            rospy.set_param('~model', self.MODEL)
        else:
            raise EnvironmentError('No model parameter defined')

        self.setWindowTitle('Video Feed: ' + self.MODEL)

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
        self.subVideo = rospy.Subscriber('ardrone/image_raw', Image, self.ReceiveImage)

        '''
        self.takeoff = QtGui.QCommandLinkButton('Take Off', self)
        self.land = QtGui.QCommandLinkButton('Land', self)
        self.DNN = QtGui.QCommandLinkButton('DNN Toggle', self)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.takeoff)
        self.layout.addWidget(self.land)
        self.layout.addWidget(self.DNN)
        
        self.takeoff.clicked.connect(self.SendTakeoff)
        self.land.clicked.connect(self.SendLand)
        self.DNN.clicked.connect(self.SendToggle)
        
        self.setGeometry(530, 320, 300, 200)
        self.setLayout(self.layout)
        '''
        # Holds the status message to be displayed on the next GUI update
        self.statusMessage = ''

        # Tracks whether we have received data since the last connection check
        # This works because data comes in at 50Hz but we're checking for a connection at 4Hz
        self.communicationSinceTimer = False
        self.connected = False

        # A timer to check whether we're still connected
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)


    # Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False
   
    def ReceiveNavdata(self, navdata):
        # Indicate that new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # Update the message to be displayed
        msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        self.statusMessage = '{} (Battery: {}%)'.format(msg, int(navdata.batteryPercent))

        self.tagLock.acquire()
        try:
            if navdata.tags_count > 0:
                self.tags = [(navdata.tags_xc[i], navdata.tags_yc[i], navdata.tags_distance[i]) for i in
                             range(0, navdata.tags_count)]
            else:
                self.tags = []
        finally:
            self.tagLock.release()

if __name__ == '__main__':
    import sys

    rospy.init_node('ardrone_video_display')
    app = QtGui.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
