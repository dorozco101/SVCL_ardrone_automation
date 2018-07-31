#!usr/bin/env python

import rospy
import cv2
import time
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController
from os.path import expanduser
from std_msgs.msg import Int32, Float32
from svcl_ardrone_automation.msg import *

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class FaceDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over. Altitude is maintained
    def __init__(self):
        
        #self.Kp,self.Ki,self.Kd = 0.11,0.0,0.0004
        #self.Kp,self.Ki,self.Kd = 0.1,20.0,0.0005 #best
        self.processVideo = ProcessVideo()

    # given the image and navdata of the drone, returns the following in order:
    #
    # A directive status int:
    #   0 if algorithm is still running and drone isn't on orange yet
    #   1 if algorithm is finished and drone is now on orange
    #
    # A tuple of (xspeed, yspeed, yawspeed, zspeed):
    #   indicating the next instructions to fly the drone
    #
    # An image reflecting what is being done as part of the algorithm
    def RetrieveNextInstruction(self, image, navdata):

        distance, center = self.processVideo.DetectFaces(image)
        if distance != None:
            rospy.logwarn("Distance to face: "+str(distance/1000.0))
        directiveStatus = 0
        return directiveStatus, (0, 0, 0, 0), image, None,0, 0,None


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        self.Reset()

    def Reset(self):
        pass


