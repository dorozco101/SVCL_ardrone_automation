#!/usr/bin/env python

import sys
import rospy
import message_filters
import numpy as np
import os.path
import time
#Import messages we want for recieving video feed
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tum_ardrone.msg import *
from ardrone_autonomy.msg import Navdata
from processing_functions import *
#Import service type for toggle cam
from std_srvs.srv import Empty

# Import cv2 as GUI
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DroneVideo(object):

    def __init__(self):
        
        super(DroneVideo, self).__init__()

        self.bridge=CvBridge()
        self.cv_image=None
        self.windowName = "Live AR.Drone Video Stream"
        self.infoWindow = "Flight Info"
        self.info = np.zeros((70,100,3), np.uint8)

        #Subscribe to the drones video feed
        self.video = rospy.Subscriber('/ardrone/image_raw', Image, self.ROStoCVImage )
        #self.video = message_filters.Subscriber('/ardrone/image_raw', Image)

        self.state = rospy.Subscriber('/ardrone/predictedPose',filter_state,self.callback)
        #self.state = message_filters.Subscriber('/ardrone/camera_info',CameraInfo)
        self.moved = False
        self.currentTime = 0
        self.tracker = Tracker()
        self.lastTime = 0
        self.window = [0,0,0,0,0,0,0,0,0]

    def ROStoCVImage(self,data):
        
        #convert ROS Image to OpenCV Image
        #rospy.logwarn(state)
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.ShowVideo()
        
    def callback(self,state):
        self.translation = [state.x,state.y,state.z]
        self.yaw = -state.yaw
        self.roll = state.roll
        self.pitch = state.pitch
        #rospy.logwarn(str(self.x)+str(self.y)+str(self.z))
        
        self.tracker.update(self.roll,self.pitch,self.yaw,self.translation)

    def ShowVideo(self):
        
        self.KeyListener()

        self.ReceivedVideo()
        self.lastTime = self.currentTime        
        self.currentTime = time.time()
        dtime = self.currentTime-self.lastTime*1000.0
        sum = 0
        for i in range(len(self.window)-1):
            self.window[i]=self.window[i+1]
            sum += self.window[i]
        sum=sum/(len(self.window)-1)
            
        self.window[len(self.window)-1]=dtime
        
        cv2.imshow(self.windowName, self.cv_image)
        cv2.imshow(self.infoWindow, self.info)
        cv2.waitKey(3)

        # move the GUI into the middle of the screenbefore the first frame is shown
        if self.moved == False :
            cv2.moveWindow(self.windowName, 750, 500)
            cv2.moveWindow(self.infoWindow, 750, 500)
            self.moved = True
        

    # processes the video before it is shown
    # don't implement here; implement in subclasses (TraceCircleController)
    def ReceivedVideo(self):
        pass

    #defines any keys to listen to
    # don't implement here; implement in subclasses (TraceCircleController)
    def KeyListener(self):
        pass
    
if __name__=='__main__':
    
    display=DroneVideo()
    rospy.init_node('DroneVideo')
    rospy.spin()
cv2.destroyAllWindows()
