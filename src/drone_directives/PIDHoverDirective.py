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
import numpy as np
# describes instruction on what the drone should do in order to hover over 
# a specified color underneath it
class PIDHoverDirective(AbstractDroneDirective):
    
    # sets up this directive
    # platformColor: color to hover over. Altitude is maintained
    def __init__(self, poseTracker,target,waitDist=0.05):
        
        #self.Kp,self.Ki,self.Kd = 0.1,20.0,0.0005 #best
        self.Kp,self.Ki,self.Kd = 0.11,0.0,0.0005
        #self.Kp,self.Ki,self.Kd = 0.0001,0.0,0.0000
        self.moveTime = 0.2
        self.waitTime = 0
        self.tracker = poseTracker
        self.target = target
        self.waitDist = waitDist
        self.worldTarget = self.tracker.body2World(target)[:,0]
        self.processVideo = ProcessVideo()
        self.centery = 360/2.0
        self.centerx = 640/2.0
        self.pub = rospy.Publisher('ardrone/tracker',tracker)
        self.track = tracker()
        self.platform = [0,0,0]
        self.buff = np.repeat(np.asarray([self.worldTarget]).T,50,axis=1)
        self.tapeCounter = 0
        self.tapeLocation = None
        #the amount of weight we would like to put towards correcting the drones drift by recognizing landmarks
        self.correctionRatio = 0.3
    def distance(self,x,y):
        dist = (x[0]-y[0])**2+(x[1]-y[1])**2
        dist = dist**(0.5)
        return dist
    def weightedUpdate(self,prediction,updateTerm):
        return (self.correctionRatio*updateTerm[0,0]+(1-self.correctionRatio)*prediction[0,0],
            self.correctionRatio*updateTerm[1,0]+(1-self.correctionRatio)*prediction[1,0],updateTerm[2,0],1.0)
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

        segImage, radius, center = self.processVideo.RecognizeShape(image, 'orange',(None,None))
        self.yaw = self.tracker.yaw
        loc = (0,0,0,0)
        #circle detection
        rospy.logwarn("x: "+str(self.tracker.translation[0])+" y: "+str(self.tracker.translation[1]))
        if radius != None:
            predictedZ = self.processVideo.CalcDistanceNew(88.0, radius* 2)/1000.0
            scale = (88.0/(radius*2))/1000.0 #meters/pixel
            x = (center[0]-self.centerx)*scale
            y = (self.centery-center[1])*scale
            
            tape = self.tracker.camera2Body([x,y,-predictedZ])
            worldPoint = self.tracker.camera2World([x,y,-predictedZ])
            
            if ( self.tapeCounter < 50 and self.distance(worldPoint,self.worldTarget) < 0.25):
                self.tapeCounter+=1
                rospy.logwarn("Entering if " +str(self.tapeCounter))
                if self.tapeCounter == 50:
                   self.tapeLocation = self.worldTarget
                for i in range(49):
                    self.buff[:,i] = self.buff[:,i+1]
                self.buff[:,49] = np.asarray([worldPoint[0,0],worldPoint[1,0],worldPoint[2,0]])
                self.worldTarget = np.mean(self.buff,1)

            if self.tapeLocation != None:
                dist = self.distance(worldPoint,self.tapeLocation)
                if dist < 0.35 and dist > 0.15:
                    loc = self.tracker.tape2World([x,y,-predictedZ],self.yaw,[self.tapeLocation[0],self.tapeLocation[1],0])
                    loc = self.weightedUpdate(worldPoint,loc)
                    rospy.logwarn("Fixing location to ..."+str(loc))
            
            self.track.landMark = (self.worldTarget[0],self.worldTarget[1],0.0,1.0)
        else:
            self.track.landMark = (0,0,0,0.0)
        rospy.logwarn("world target: " + str(self.worldTarget))
        self.track.landMark = (self.worldTarget[0],self.worldTarget[1],0.0,1.0)
        self.track.loc = loc
        self.pub.publish(self.track)
        self.currentTarget = self.tracker.world2Body(self.worldTarget)
        self.currentTime = time.time()
        
        if self.lastTime == 0:
            self.rollError = 0
            self.pitchError = 0
        else:
            self.rollError = self.currentTarget[0]
            self.pitchError = self.currentTarget[1]

        self.dt = (self.currentTime - self.lastTime)/1000.

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
        #rospy.logwarn("roll: "+str(self.tracker.roll))
        #rospy.logwarn("pitch: "+str(self.tracker.pitch))
        #rospy.logwarn(directiveStatus)
        return directiveStatus, (roll, pitch, 0, 0), segImage, None,self.moveTime, self.waitTime,None


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


