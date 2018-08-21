#!/usr/bin/env python

import sys
import rospy
import message_filters
import numpy as np
import os.path
import time
#Import messages we want for recieving video feed

from svcl_ardrone_automation.msg import *
from ardrone_autonomy.msg import Navdata
from processing_functions import *


class DroneTracker(object):

    def __init__(self):
        
        super(DroneTracker, self).__init__()
        self.stateSub = rospy.Subscriber('/ardrone/predictedPose',filter_state,self.callback)
        self.tracker = Tracker()
        self.trackerPub = rospy.Publisher('ardrone/tracker',tracker)

    def callback(self,state):
        #self.translation = [1.75*state.x,1.4*state.y,state.z]
        self.translation = [state.x*1.1,state.y*1.1,state.z]
        self.yaw = -state.yaw
        self.roll = state.roll
        self.pitch = state.pitch
        self.tracker.update(self.roll,self.pitch,self.yaw,self.translation)
