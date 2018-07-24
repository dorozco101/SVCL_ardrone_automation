#!usr/bin/env python

import rospy
import cv2
import time
from processing_functions.process_video import ProcessVideo
from AbstractDroneDirective import *
from processing_functions.pid_controller import PIDController
from os.path import expanduser
from std_msgs.msg import Int32, Float32

# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over. Altitude is maintained
    def __init__(self, tracker,target,waitDist=0.1):
        
        #self.Kp,self.Ki,self.Kd = 0.8,-20.0,0.004
        self.Kp,self.Ki,self.Kd = 0.1,20.0,0.0005
        self.tracker = tracker
        self.target = target
        self.waitDist = waitDist
        self.worldTarget = self.tracker.body2World(target)[:,0]
        #self.pub_pid_xspeed = rospy.Publisher('pid_xspeed', Float32, queue_size = 10)
        #self.pub_pid_yspeed = rospy.Publisher('pid_yspeed', Float32, queue_size = 10)
        #self.pub_pid_in_alt = rospy.Publisher('pid_in_alt', Int32, queue_size = 10)
        #rate = rospy.Rate(5)

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
        self.currentTime = time.time()
        #rospy.logwarn(self.worldTarget)
        #rospy.logwarn(type(self.worldTarget))
        self.currentTarget = self.tracker.world2Body(self.worldTarget)
        if self.lastTime == 0:
            self.rollError = 0
            self.pitchError = 0
        else:
            self.rollError = self.currentTarget[0]
            self.pitchError = self.currentTarget[1]

        self.dt = (self.currentTime - self.lastTime)/1000.
        #rospy.logwarn(str(self.dt))
        
        self.totalError = [self.totalError[0]+self.rollError*self.dt, 
                        self.totalError[1]+self.pitchError*self.dt,0]
        pRoll = -self.Kp*(self.rollError)
        iRoll = -self.Ki*(self.totalError[0])
        dRoll = -self.Kd*((self.rollError-self.lastError[0])/self.dt)

        pPitch = self.Kp*(self.pitchError)
        iPitch = self.Ki*(self.totalError[1])
        dPitch = self.Kd*((self.pitchError-self.lastError[1])/self.dt)

        self.lastError = self.currentTarget
        self.lastTime = self.currentTime
        rospy.logwarn(str(iRoll))
        rospy.logwarn(str(iPitch))
        roll = pRoll +iRoll+dRoll
        pitch = pPitch + iPitch + dPitch

        if (abs(self.rollError) <= self.waitDist and abs(self.pitchError) <=self.waitDist):
            directiveStatus = 1
        else:
            directiveStatus = 0
        #Trim commands over the drones command limit
        roll = 1 if roll>1 else roll
        roll = -1 if roll<-1 else roll
        pitch = 1 if pitch>1 else pitch
        pitch = -1 if pitch<-1 else pitch
        #rospy.logwarn(directiveStatus)
        return directiveStatus, (roll, pitch, 0, 0), image, None,0, 0,None


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        self.Reset()

    def Reset(self):
        self.dt = 0
        self.currentTime = time.time()
        self.lastTime = 0
        self.rollError = 0
        self.pitchError = 0
        self.lastError = [0,0]
        self.totalError = [0,0,0]


