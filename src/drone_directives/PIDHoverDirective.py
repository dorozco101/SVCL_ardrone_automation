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
    # plrratformColor: color to hover over. Altitude is maintained
    def __init__(self, poseTracker,target,yaw,platformNumber,waitDist=0.1):
        
        #self.Kp,self.Ki,self.Kd = 0.1,20.0,0.0005 #best
        #self.Kp,self.Ki,self.Kd = 0.2,0.0,0.0005
        self.Kp,self.Ki,self.Kd = 0.21,0.0,0.0006
        #self.Kp,self.Ki,self.Kd = 0.17,0.0,0.0004
        self.Kpz = 0.1
        self.KpYaw,self.KiYaw,self.KdYaw = (2/90.),0,0
        self.targetYaw = -yaw
        self.moveTime = 0.2
        self.waitTime = 0.0
        self.tracker = poseTracker
        self.target = target
        self.target[2] = target[2]-self.tracker.translation[2]
        self.waitDist = waitDist
        self.worldTarget = self.tracker.body2World(target)[:,0]
        self.processVideo = ProcessVideo()
        self.platformNumber = platformNumber
        self.centery = 360/2.0
        self.centerx = 640/2.0
        self.pub = rospy.Publisher('ardrone/tracker',tracker)
        self.track = tracker()
        self.platform = [0,0,0]
        self.filterSize = 30
        self.platformBuff = np.repeat(np.asarray([self.worldTarget]).T,self.filterSize,axis=1)
        self.heightBuff = np.zeros(4)
        self.worldPoint = np.asarray([[0,0,0]]).T
        self.lastLocation = self.tracker.translation
        #the amount of weight we would like to put towards correcting the drones drift by recognizing landmarksgggs
        self.correctionRatio = 0.9999
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
        #algTime = time.time()
        image = cv2.inRange(image,np.array([200,200,200]),np.array([255,255,255]))
        row,col = image.shape
        img,circles = self.processVideo.detectCircles(image)
        if len(circles) != 0:
            circle = circles[0]
            center = circle[0]
            radius = circle[1]
            lineWidth = (2*radius)*(40.0/88.0)
            image,lines = self.processVideo.detectLines(image,int(lineWidth))
            image = cv2.bitwise_or(image,img)

        bestTheta = None
        if len(circles) != 0:
            minDist = -1
            for circle in circles:
                radius = circle[1]
                center = circle[0]
                predictedZ = self.processVideo.CalcDistanceNew(88.0, radius* 2)/1000.0
                scale = (88.0/(radius*2))/1000.0 #meters/pixel
                x = (center[0]-self.centerx)*scale
                y = (self.centery-center[1])*scale
            #rospy.logwarn(self.tapeLocation)
            #tape = self.tracker.camera2Body([x,y,-predictedZ])
                point = self.tracker.camera2World([x,y,-predictedZ])
                dist = self.distance(point,self.worldTarget)
                if dist < minDist or minDist == -1:
                    worldPoint = point
                    z = predictedZ
                    Center = center
            minDist = -1
            for line in lines:
                line = line[0]
                rho = line[0]
                theta = line[1]
                dist = self.processVideo.line2PointDist(rho,theta,Center)
                if minDist == -1 or dist<minDist:
                    minDist = dist
                    bestTheta = theta
        if bestTheta != None:
            self.currentYaw = bestTheta
        else:
            self.currentYaw = 0
        theta = self.currentYaw
         #Calculate closest rotation to get to target angle
        #theta = (self.currentYaw+360)%360
        #theta = (theta-360) if (theta >180) else theta

        zError = 0
        loc = (0,0,0,0)
        
        #circle detection
        #rospy.logwarn("x: "+str(self.tracker.translation[0])+" y: "+str(self.tracker.translation[1]))
        if len(circles) != 0:
            self.worldPoint = worldPoint
            rospy.logwarn(self.worldPoint)
            if ( self.distance(worldPoint,self.worldTarget) < 0.35):
                for i in range(self.heightBuff.shape[0]-1):
                    self.heightBuff[i] = self.heightBuff[i+1]
                self.heightBuff[self.heightBuff.shape[0]-1] = z
                height = np.mean(self.heightBuff)
                zError = (self.worldTarget[2]-height)
                for i in range(self.filterSize-1):
                    self.platformBuff[:,i] = self.platformBuff[:,i+1]
                self.platformBuff[:,self.filterSize-1] = np.asarray([worldPoint[0,0],worldPoint[1,0],self.worldTarget[2]])
                self.worldTarget = np.mean(self.platformBuff,1)
            self.track.landMark = (self.worldTarget[0],self.worldTarget[1],0.0,1.0)
        else:
            self.track.landMark = (0,0,0,0.0)
            rospy.logwarn("not seen")

        #rospy.logwarn("world target: " + str(self.worldTarget))
        self.track.landMark = (self.worldTarget[0],self.worldTarget[1],0.0,1.0)
        self.track.loc = loc
        self.track.yaw = (self.targetYaw,0.0)
        self.pub.publish(self.track)
        self.currentTarget = self.tracker.world2Body(self.worldTarget)
        self.currentTime = time.time()
        
        if self.lastTime == 0:
            self.rollError = 0
            self.pitchError = 0
            self.zError = 0
            self.yawError = 0
        else:
            self.rollError = self.currentTarget[0]
            self.pitchError = self.currentTarget[1]
            self.zError = zError
            self.yawError = theta

        if (self.tracker.translation[0] == self.lastLocation[0] and self.tracker.translation[1] == self.lastLocation[1] and self.lastLocation[2] == self.tracker.translation[2]):
            #self.rollError = 0
            #self.pitchError = 0
            rospy.logwarn("Error: State estimation may be crashed")
        self.dt = (self.currentTime - self.lastTime)/1000.

        self.totalError = [self.totalError[0]+self.rollError*self.dt, 
                        self.totalError[1]+self.pitchError*self.dt,
                        self.totalError[2]+self.yawError*self.dt,0]

        pRoll = -self.Kp*(self.rollError)
        iRoll = -self.Ki*(self.totalError[0])
        dRoll = -self.Kd*((self.rollError-self.lastError[0])/self.dt)

        pPitch = self.Kp*(self.pitchError)
        iPitch = self.Ki*(self.totalError[1])
        dPitch = self.Kd*((self.pitchError-self.lastError[1])/self.dt)

        pYaw = self.KpYaw*(self.yawError)
        iYaw = self.KiYaw*(self.totalError[2])
        dYaw = self.KdYaw*((self.yawError-self.lastYawError)/self.dt)

        self.lastError = self.currentTarget
        self.lastYawError = self.yawError

        pHeight = self.Kpz*(self.zError)

        self.lastTime = self.currentTime
        
        roll = pRoll +iRoll+dRoll
        pitch = pPitch + iPitch + dPitch
        yaw = pYaw + iYaw + dYaw

        zVel = pHeight
        
        if (abs(self.rollError) <= self.waitDist and abs(self.pitchError) <=self.waitDist and abs(self.zError) <= self.waitDist) and abs(self.yawError) <= 6:# and len(circles) != 0):
            directiveStatus = 1
        else:
            directiveStatus = 0
        #Trim commands over the drones command limit
        roll = 1 if roll>1 else roll
        roll = -1 if roll<-1 else roll
        pitch = 1 if pitch>1 else pitch
        pitch = -1 if pitch<-1 else pitch
        yaw = -1 if yaw <-1 else yaw
        yaw = 1 if yaw>1 else yaw
        zVel = -1 if zVel <-1 else zVel
        zVel = 1 if zVel>1 else zVel

        #rospy.logwarn("roll: "+str(self.tracker.roll))
        #rospy.logwarn("pitch: "+str(self.tracker.pitch))
        #rospy.logwarn(directiveStatus)
        #rospy.logwarn(self.zError)
        #rospy.logwarn("zError: "+str(self.zError))
        #rospy.logwarn("yaw: " +str(self.currentYaw))
        rospy.logwarn("yawErr: "+str(self.yawError))
        #rospy.logwarn("algTime: "+str(time.time()-algTime))
        if abs(self.rollError) > self.waitDist or abs(self.pitchError) > self.waitDist:
            yaw = 0
            rospy.logwarn("not close enough to adjust yaw")
        return directiveStatus, (roll, pitch, yaw, zVel), image, None,self.moveTime, self.waitTime,None


    # This method is called by the state machine when it considers this directive finished
    def Finished(self):
        self.Reset()
        #tapeLocation = self.tracker.body2World(self.target)[:,0]
        #loc = self.tracker.tape2World([x,y,-predictedZ],self.yaw,[tapeLocation[0],tapeLocation[1],0])
        #if (self.platformNumber%2==0):
        loc = np.asarray([self.target]).T
        loc[2] = 1.2
        loc = (loc[0],loc[1],loc[2],1.0)
        rospy.logwarn("Reseting location to" +str(loc))
        #loc = self.weightedUpdate(self.worldPoint,loc)
        self.track.loc = loc
        self.track.yaw = (self.targetYaw,1.0)
        self.pub.publish(self.track)

    def Reset(self):
        self.dt = 0
        self.currentTime = time.time()
        self.lastTime = 0
        self.rollError = 0
        self.pitchError = 0
        self.lastError = [0,0,0]
        self.lastYawError = 0
        self.totalError = [0,0,0,0]


