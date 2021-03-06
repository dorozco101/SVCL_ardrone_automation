#!/usr/bin/env python

import sys
import rospy

#import opencv and numpy for image processing
import cv2
from numpy import *
import time
from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser
from random import randint
from math import *

# This class contains helper functions that each process
# video frames in some way
class ProcessVideo(object):


#The purpose of this function is to segement an image by setting all of the pixels an image
#outside of a specified color range to 0, only leaving the desired color in the image.

#param2::image: the image we would like to segment, expected to be bgr
#param3::color: string specifying the color you would like to keep. These values are defined below
#param4::returnType: string to specify the type of image, you would like this function to return.
#by default will return a segmented image in BGR, this can also be specified to return following.
#"binary" returns a binary image, 1 is region within color range
#"hsv" returns an hsv image, of pixels within color range

    def DetectColor(self,image,color,returnType = "segmented", process = False):
        
        numRows,numCols,channels=image.shape
        lower2=upper2=array([0,0,0])

	#definitions for upper and lower hsv values for each color
        if(color=='orange'): #0,50,170,10,254,255
            # for when lighting is dim
            #s_min = 50
            #s_max = 254
            #for when lighting is bright
            """
            s_min = 94
            s_max = 255
            hsv_boundaries = [( [0, s_min, 170],[10, s_max, 255] )]
            hsv_boundaries2 = [([174, s_min, 180],[180, s_max, 255])]
            """
            hsv_boundaries = [( [0, 30, 40],[25, 255, 160] )]
            hsv_boundaries2 = [([160, 30, 40],[180, 255, 160])]
            #hsv_boundaries2 = [([174, 61, 120],[180, 255, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        elif(color=='front orange'):
            #0 50 0 25, 254 255
            hsv_boundaries = [( [0, 55, 10],[60, 255, 255] )]
            #168 50 0,180 254 255
            #lower  hsv boundary #170 140 150,179 255 255
            hsv_boundaries2 = [([168, 70, 10],[180, 254, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        elif(color=='blue'):
            hsv_boundaries = [ ([100,50,70],[140,255,255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        elif(color=='darkblue'):
            hsv_boundaries = [ ([90,25,70],[135,255,250])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        elif(color=='green'):
            hsv_boundaries = [ ([40, 70, 10],[70, 190, 254])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")

        elif(color=='white'):
            #duct tape
            #hsv_boundaries = [ ([161, 68, 127],[171, 209, 255])]
            # printed
            hsv_boundaries = [ ([10,10,10 ],[160,160,160])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        elif(color=='pink'):
            #duct tape
            #hsv_boundaries = [ ([161, 68, 127],[171, 209, 255])]
            # printed
            hsv_boundaries = [ ([161, 68, 127],[172, 255, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        elif(color=='yellow'):
            hsv_boundaries = [ ([17, 10, 100],[32, 188, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
        elif(color =='test'):
            hsv_boundaries = [( [1, 90, 120],[45, 255, 255] )]
            #hsv_boundaries2 = [([178, 75, 120],[180, 255, 255])]
            #hsv_boundaries2 = [([174, 61, 120],[180, 255, 255])]
            lower=array(hsv_boundaries[0][0], dtype = "uint8")
            upper= array(hsv_boundaries[0][1],dtype = "uint8")
            #lower2=array(hsv_boundaries2[0][0], dtype = "uint8")
            #upper2=array(hsv_boundaries2[0][1], dtype = "uint8")

        else:
            raise Exception("Color not recognized")

	#convert bgr to hsv image for color segmentation
        if color == 'white':
            hsv_image = image
        else:
            hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #find colors within the boundaries for each pixel,returns binary image
        mask = mask1 = cv2.inRange(hsv_image,lower,upper)

        # if a second bound exists, we combine the two masks
        if( lower2.any() or upper2.any() ):
            mask2 = cv2.inRange(hsv_image,lower2,upper2)
	    #perform logical or to combine the masks
            mask = cv2.bitwise_or(mask1,mask2,mask=None)
        
	#this sets any pixel in mask != 0 to its hsv color value from unsegmented image
        hsv_output = cv2.bitwise_and(hsv_image,hsv_image, mask = mask)
        #this converts the image from hsv to bgr
        if color == 'white':
            segmentedImage = hsv_output
        else:
            segmentedImage = cv2.cvtColor(hsv_output,cv2.COLOR_HSV2BGR)
        #we put a circle in the center of the image 
        #cv2.circle(segmentedImage,(numcols/2,numrows/2),4,150,1)
        
        #rospy.logwarn(hsv_image[numRows/2][numCols/2])
        #rospy.logwarn("seg: " + str(hsv_output[numRows/2][numCols/2]))
        #segmentedImage is bgr, and mask is a binary image with values within color range
        if(returnType == "segmented"):
            return segmentedImage
        elif returnType == "hsv":
            return hsv_output
        elif returnType == "binary":
            return mask
        elif returnType == "all":
            return segmentedImage,hsv_output,mask


    def RemoveNoise(self, image, size = 5):
        processedImg = image.copy()
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        #processedImg = cv2.morphologyEx(processedImg, cv2.MORPH_OPEN, kernel)
        #segmentedImage = cv2.morphologyEx(segmentedImage, cv2.MORPH_OPEN, kernel)
        #segmentedImage = cv2.morphologyEx(segmentedImage, cv2.MORPH_CLOSE, kernel)
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size,size))
        processedImg= cv2.erode(processedImg, kernel2, iterations = 1)

        return processedImg


    #a non classical more accurate model for calculating distance,object true size expected in mm
    #and returns distance in mm
    #old val = 781.6, =-319.4
    def CalcDistanceNew(self,objectTrueSize,objectPixels,focalLength = 715.6186, offset = 5.1371):
        distance = ( (focalLength*objectTrueSize)/(objectPixels))+offset
        return distance
    
    def calcScale(self,objectTrueSize,distance,focalLength = 715.6186,offset = 5.1371):
        objectPixels = (focalLength*objectTrueSize)/(distance-offset)
        return objectPixels
    def CircleScale(self,objectPixels,objectTrueSize):
            return (float(objectTrueSize)/objectPixels)
    #calculates the focal length of a flat lense, given as parameters: an objects apparent size in pixels,
    #an objects true size in mm, and the distance from the object in mm
    def CalcFocal(self,objectPixelSize,objectTrueSize,distance):
        focal = (objectPixelSize*distance)/trueObjectSize
        return focal

    # returns an array of all lines from segmented image, each with a center and angle.
    # The first tuple is always the middle line to use (closest to horizontal)
    def MultiShowLine(self, image, sort = True):

       # turning segmented image into a binary image and performing a close on it
        processedImg = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
        _, processedImg = cv2.threshold(processedImg, 15, 255, 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        processedImg = cv2.morphologyEx(processedImg, cv2.MORPH_CLOSE, kernel)

        # finding and drawing contours onto the image
        _, contours, _ = cv2.findContours(processedImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)

        #drawImg = cv2.cvtColor(processedImg, cv2.COLOR_GRAY2BGR)
        drawImg = image
        drawn = 0
        centers = []
        lines = []

        for c in contours:
            
            if cv2.contourArea(c) > 170:

                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box1 = int0(box)
                #cv2.drawContours(drawImg, [box1], 0, (0,0,255),2)
                # finding longest line
                longest = ( (0,0), (0,0), 0)
                for i in range(len(box)-1):
                    lineLen = math.sqrt(math.pow((box[i+1][0] - box[i][0]),2) 
                    + math.pow((box[i+1][1] - box[i][1]),2) ) 
                    if lineLen > longest[2]:
                        longest = ( ((box[i+1][0]),(box[i+1][1])), ((box[i][0]),(box[i][1])), lineLen)
                cv2.line(image, longest[0], longest[1], (255,255,255),3)
                vert = longest[1][1] - longest[0][1]
                horiz = longest[1][0] - longest[0][0]
                
                if horiz !=0:
                    angle = rad2deg(arctan(vert/horiz))
                    angle = angle+90
                    if angle > 90:
                        angle = angle-180
                else: 
                    angle = 0
                #if angle != 0:
                #angle = -angle
                
                #if angle < 0:
                    #angle = angle + 180
                # finding the center
                M = cv2.moments(c)

                if M["m00"]!=0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
    
                    # drawing center and contour on image
                    cv2.drawContours(drawImg, [c], -1, (255, 191, 30), 4)
                    cv2.circle(drawImg, (cX, cY), 7, (0,255,0), -1)
                    
                    # line angle, center, endpoint 1, endpoint 2, length
                    lines.append((angle, (cX,cY), longest[0], longest[1], longest[2]))

                drawn += 1
        # to do, debt
        if sort == False:
            return lines, drawImg
        else:
            # finding middle line, closest to the orientation
            lines = sorted(lines, key = self.getHoriz)
            return lines, drawImg


    def getHoriz(self, line):
        angle = line[0]
        if angle < 180 - angle:
            return angle
        else:
            return 180-angle

    def getVert(self, line):
        angle = line[0]
        return abs(90-angle)


    def ShowTwoLines(self, image):
        
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur( gray, (7,7),0)

        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        lines = cv2.HoughLinesP(edges,1, pi/180, 25, minLineLength = 10, maxLineGap = 380)
        #lines = cv2.HoughLinesP(edges,1, pi/180, 55, minLineLength = 10, maxLineGap = 380)
                
        line1List = []
        line2List = []
        line1 = None
        line2 = None
        line1Angle= None
        line1Center = None
        line2Angle= None
        line2Center = None

        if lines != None:
            
            # trimming
            temp = []
            for line in lines:
                temp.append(line[0])
            lines = temp
            
            #for line in lineList:
                #cv2.line(image,(line[0],line[1]),(line[2],line[3]), (0,randint(0,255),randint(2,255)),1)

            # tuple in format (line, slope)
            line1List.append(lines[0])
            # sometimes the angle comes out at -0.0; adding 0 fixes that to regular 0.0
            compAngle = self.GetAngle(line1List[0])
            compAngle = compAngle + 0

            degThresh = 45

            # Finding the best two distinct lines from the array
            for i in range( 1, len(lines) ):
                
                angle = self.GetAngle(lines[i])
                angle = angle + 0
                
                if ( (abs(angle - compAngle) < degThresh) or
                (  (180 - (degThresh/2)) < compAngle and angle < (degThresh/2) ) or
                ( compAngle < (degThresh/2) and (180-compAngle) < (degThresh/2) ) ): 

                    line1List.append(lines[i])
                else:
                    line2List.append(lines[i])

            line1 = self.FindAvgLine(line1List)
            line1Angle = self.GetAngle(line1)
            line1Center = ( ((line1[0]+line1[2])/2), ((line1[1] + line1[3])/2) )
            line2 = self.FindAvgLine(line2List)
            line2Angle = self.GetAngle(line2)

            if line2!= None:

                line2Center = ( ((line2[0]+line2[2])/2), ((line2[1] + line2[3])/2) )
                # line 1 is always the line closer to horizontal
                if abs(line2Angle-90) > abs(line1Angle-90):
                    temp1, temp2, temp3 = line1, line1Angle, line1Center
                    line1, line1Angle, line1Center = line2, line2Angle, line2Center
                    line2, line2Angle , line2Center = temp1, temp2, temp3

                cv2.line(image,(line2[0],line2[1]),(line2[2],line2[3]), (0,0,255),3)
                #rospy.logwarn("line 1 = " + str(line1Angle) + ", line 2 = " + str(line2Angle))

            cv2.line(image,(line1[0],line1[1]),(line1[2],line1[3]), (0,255,0),3)

        return line1Angle, line1Center, line2Angle, line2Center

    
    def GetAngle(self, line):

        if line != None:
            # checking for a vertical line
            if ( line[2] - float(line[0]) ) != 0:
                slope = ( line[3] - float(line[1]) ) / ( line[2] - float(line[0]) ) 
                angle = rad2deg(arctan(slope))
                #angle = 180 - arctan(slope)
            else:
                angle = 90
            
            angle = -angle
            if angle < 0:
                angle = angle + 180

            return angle
        else:

            return None


    # Helper method
    def FindAvgLine(self, linesList):
        
        if len(linesList) != 0:

            P1xSum = 0
            P1ySum = 0
            P2xSum = 0
            P2ySum = 0
            
            for line in linesList:
                P1xSum += line[0]
                P1ySum += line[1]
                P2xSum += line[2]
                P2ySum += line[3]

            line = ( int(P1xSum/len(linesList)), int(P1ySum/len(linesList)),
            int(P2xSum/len(linesList)), int(P2ySum/len(linesList)) )

            return line

        else:

            return None
        

    # Performs houghline transform to detect lines by inputing a BGR image and returning
    # the angle of the line and the point perpendicular to the origin
    def ShowLine(self,image, lowerAngleBound = 0, upperAngleBound = 180, secondBounds = (None,None), thresh=65):
        
        #change bgr to gray for edge detection

        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur( gray, (7,7),0)

        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        lines = cv2.HoughLines(edges,1, pi/180, thresh)

        """
        while(type(lines)==type(None) and thresh > 30):
            lines = cv2.HoughLines(edges,1,pi/180,thresh)
            thresh-=1
        """
        #average rho and theta values
        if(lines!= None):
        
            Lines=array(lines)
            thetas=Lines[:,:,1]
            rhos=Lines[:,:,0]
            thetasDegrees = (thetas*180)/pi

            #rospy.logwarn(thetasDegrees)

            #boolean arrays for angles greater than 170 and less than 10
            large = (thetasDegrees>170)
            small = (thetasDegrees<10)
            extract = logical_and((thetasDegrees > lowerAngleBound),(thetasDegrees < upperAngleBound))
            #rospy.logwarn("first bound:" + str(extract))

            if(secondBounds != (None,None)):
                extractSecond = logical_and((thetasDegrees > secondBounds[0]),(thetasDegrees < secondBounds[1]))
                extract = logical_or(extract,extractSecond)
            
                #rospy.logwarn("second bound:"+str(extractSecond))
            #rospy.logwarn("combined:" + str(extract))

            #if most lines are within this range of theta
            
            if( sum(large | small) > (size(thetas)/2) and (lowerAngleBound <= 0)):
                if(sum(large) > sum(small)):
                    #sums up all angles greater than 170 and averages them
                    radians = sum(thetas*large)/(sum(large))
                    rho = sum(rhos*large)/(sum(large))


                else:
                    #adds up all angles less than 10 and averages them
                    radians = sum(thetas*small)/(sum(small))
                    rho = sum(rhos*small)/(sum(small))
  
            else:
                #takes average of all angles
                temp=(thetas*extract)*180/pi
                #rospy.logwarn(temp)
                radians = sum(thetas*extract)/(sum(extract))
                rho = sum(rhos*extract)/(sum(extract))
                

                #LINES = matrix(lines).mean(0)
                #rho=LINES[0,0]
                #radians=LINES[0,1]
            # all data needed to plot lines on original image
            
            if( not math.isnan(radians)):
                a = cos(radians)
                b = sin(radians)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
    
                cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
            #this is the correct angle relative to standard cordinate system for average line
                angle=((radians*180)/pi)
                radians=radians/pi
                #rospy.logwarn(angle)

            else: 
                angle = None
        else:

            x0 = None
            y0 = None
            angle = None
            rho = None
            radians = None

        return angle


    # Takes in a segmented image input and returns a tuple in the form (x,y),
    # where x and y are the center of mass.
    # If it does not exist, (None,None) is returned
    def CenterOfMass(self,image):

        numrows,numcols,channels=image.shape

        centerx=numcols/2
        centery=numrows/2

        #compute the center of moments for a single-channel gray image
        M=cv2.moments(cv2.cvtColor(cv2.cvtColor(image,cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY),True) 

        if M["m00"]!=0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            #draw circle on cx and cy only if center of mass exists
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)
        
            return(cx,cy)
        else:
            return (None, None)


    # Takes in a segmented image input and returns an array of tuple(s) in the form (x,y),
    # where x and y are the center of mass of each detected shape.
    # If it does not exist, an empty array is returned
    def MultiCenterOfMass(self, image):
        
        # turning segmented image into a binary image and performing a close on it
        processedImg = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
        _, processedImg = cv2.threshold(processedImg, 5, 255, 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        processedImg = cv2.morphologyEx(processedImg, cv2.MORPH_CLOSE, kernel)
        drawImg = cv2.cvtColor(processedImg, cv2.COLOR_GRAY2BGR)

        # finding and drawing contours onto the image
        _, contours, _ = cv2.findContours(processedImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        drawn = 0
        centers = []
        for c in contours:
            
            if cv2.contourArea(c) > 105:

                # finding the center
                M = cv2.moments(c)

                if M["m00"]!=0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    centers.append((cX,cY))

                    # drawing center and contour on image
                    cv2.drawContours(drawImg, [c], -1, (0, 140, 255), 4)
                    cv2.circle(drawImg, (cX, cY), 7, (0,255,0), -1)

                drawn += 1
        
        return centers, drawImg


    # takes in an image and the center of masses for its segmented version, 
    # returns how much the drone should move in the (x,y) direction such that
    # object stays in middle, within +/- tolerance pixels of the center
    # ztolerance is a tuple of (lower bound tolerance, upper bound tolerance)


    # returns yawspeed, keeps blue line horizontal in bottom cam, yspeed keeps line in middle
    # keep blue line between +/- thresh of 90 degrees (considered perfect at 90 degrees)
    # returns None if no yawspeed could be calculated
    def LineOrientation(self, image, angle, thresh, yawspeed = 0.4):

        if angle == None:
            return None

        numrows, numcols, _ = image.shape
        centerx = numcols/2
        centery = numrows/2
        upperAngle = 90 + thresh
        lowerAngle = 90 - thresh

        # Drone rotates Counter Clock_Wise
        if angle < lowerAngle and angle > 0:
            yawSpeed = yawspeed

        # Drone rotates Clock_Wise
        elif angle > upperAngle and angle < 180:
            yawSpeed = -1 * yawspeed

        # Drone is at the right angle; no need to rotate 
        else:
            yawSpeed = 0
        
        return yawSpeed
    

    # return yawspeed keeps blue line vertical in bottom cam, xspeed keeps line in middle
    # keeps line between +/- thresh from 0 degrees (perfect at 0 degrees)
    # returns None if no yawspeed could be calculated
    def ObjectOrientation(self, image, angle, thresh, yawspeed = 0.4):
        
        if angle == None:
            return None

        numrows, numcols, _ = image.shape
        centerx = numcols/2
        centery = numrows/2
        upperAngle = 180 - thresh
        lowerAngle = 0 + thresh
        
        # Drone rotates Counter Clock-Wise
        if angle < upperAngle and angle > 90:
            yawSpeed = yawspeed

        # Drone rotates Clock_Wise
        elif angle > lowerAngle and angle < 90:
            yawSpeed = -1 * yawspeed

        else:
            yawSpeed = 0


        return yawSpeed
        

    # given a segmented image hsvImage and a percentThreshold of 
    # what percentage of that image should be between hues hueMin and hueMax,
    # returns a boolean of whether or not the hsvImage and has enough of that
    # hue in it to pass that threshold 
    def IsHueDominant(self, hsvImage, hueMin, hueMax, percentThreshold):
        
        # getting array of image that considers only hue, not saturation nor value
        hsvChannels = cv2.split(hsvImage)
        hue = hsvChannels[0]
        
        # find ratio of pixels whose hue is within range, to the number of pixels overall in image
        numHuePixel = float( count_nonzero( (hueMin<hue) & (hue<hueMax) ) )
        numImagePixel= (len(hsvImage)*len(hsvImage[0]) )
        hueRatio = numHuePixel / numImagePixel

        huePercent = hueRatio * 100
        if huePercent > percentThreshold:
            return True
        else:
            return False


    # given a segmented hsv image of only 1 color, will attempt to return that
    # image with unwanted static/noise reduced
    def DeNoiseImage(self, hsvImage):
        #dst = cv2.fastNlMeansDenoisingColored(hsvImage, None, 10,10,7,5)
        dst = None
        cv2.medianBlur(hsvImage,5, dst)
        if dst == None:
            rospy.logwarn("none")
        else:
            rospy.logwarn("not none")

        return dst

    #takes in an image and the center coordinate as a tuple and simply draws a circle over the coordinate    
    def DrawCircle(self,image,center):
    
        cx = center[0]
        cy = center[1]

        
        #draw circle on cx and cy only if center is valid
        if cx != None and cy != None:
            cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1) 
            cv2.circle(image, (cx,cy), 40, 255)


    def identifyCircle(self,circle,dims,threshold):
        rows = dims[0]
        cols = dims[1]
        peri = cv2.arcLength(circle, True)
        #remove tiny noise (less than 1% of possible image)
        if peri < 0.01*(2*rows+2*cols):
            return None
        approx = cv2.approxPolyDP(circle, 0.03 * peri, True)
        if len(approx) < 5:
            return None
        M = cv2.moments(circle)
        if(M["m00"] != 0):
            cx = int(M["m10"] / M["m00"])
            cy= int(M["m01"] / M["m00"])
            #center of circle
            center = (cx,cy)
            numPoints = 0
            averageRadius = 0
            #we want to loop through every vertex on circle and measure distance to center
            for points in circle:
                point = points[0]
                #this will check if the circle is being cut off by image boundary
                if(point[0] < 0 or point[0] >= cols-0 or point[1] < 0 or point[1] >= rows-0):
                    return None
                else:
                    dist = (point - center)
                    currentRadius = sqrt(inner(dist,dist))
                    averageRadius += currentRadius
                    numPoints += 1
            averageRadius = averageRadius/numPoints
            for points in circle:
                point = points[0]
                #this will check if the circle is being cut off by image boundary
                dist = (point - center)
                currentRadius = sqrt(inner(dist,dist))
                deltaRadius = abs(currentRadius - averageRadius)
                if deltaRadius > threshold*averageRadius:
                    return None
            return (center,averageRadius)

    def detectCircles(self,img,threshold = 0.2):
        img = img.copy()
        _,possibleCircles,_ = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        for circle in possibleCircles:
            detectedCircle = self.identifyCircle(circle,img.shape,threshold)
            if detectedCircle is not None:
                circles.append(detectedCircle)
                cv2.circle(img,detectedCircle[0], int(detectedCircle[1]),150,4)
        return img,circles

    def detectLines(self,image,lineWidth):
        image = image.copy()
        edges = cv2.Canny(image,50,200,3)
        allLines = cv2.HoughLines(edges,1,pi/180, int(2*lineWidth))
        lines = []    
        if allLines is not None:
            for line in allLines:
                lines.append(line)
            
        filteredLines = []
        numLines = 0
        while len(lines) != 0:
            line = lines.pop(0)
            rho = line[0,0]
            theta = line[0,1]
            loopDist = rad2deg(theta) if rad2deg(theta) <= 90 else 180-rad2deg(theta)
            filteredLines.append([])
            filteredLines[numLines].append(line)
            pops = 0
            for index in range(len(lines)):
                otherLine = lines[index-pops]
                r = otherLine[0,0]
                t = otherLine[0,1]
                loopDist = rad2deg(t)+loopDist if rad2deg(t)<=90 else loopDist+180-rad2deg(t)
                if min(abs(rad2deg(t-theta)),abs(loopDist)) < 10 and abs(rho-r) < 2*lineWidth:
                    filteredLines[numLines].append(lines.pop(index-pops))
                    pops+=1
            numLines+=1
        for lineSet in filteredLines:
            average = zeros((1,2))
            x = 0
            y = 0
            for line in lineSet:
                average[0,0]+=line[0,0]
                theta = rad2deg(line[0,1])
                theta = deg2rad(theta*2.0)
                x += cos(theta)
                y += sin(theta)
            x = x/len(lineSet)
            y = y/len(lineSet)
            angle = rad2deg(arctan2(y,x))/2
            if angle <0:
                angle = angle+180
            average[0,1] = angle
            average[0,0] = average[0,0]/len(lineSet)
            #rospy.logwarn(average)
            lines.append(average)
            
        for line in lines:
            r = line[0,0]
            theta = deg2rad(line[0,1])
            # convert angles from -90 to 90
            #90 = clockwise, -90 counter-clockwise
            line[0,1] = 180-line[0,1]if line[0,1]%90 < line[0,1] else -line[0,1]

            a = cos(theta)
            b = sin(theta)
            x0 = a*r
            y0 = b*r
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(image,(x1,y1), (x2,y2), 150,3)
        return image,lines

    def line2PointDist(self,rho,theta,point):
        theta = -theta
        theta = deg2rad(theta+180) if theta<0 else deg2rad(theta)
        x0 = point[0]
        y0 = point[1]
        a = cos(theta)
        b = sin(theta)
        c = -rho
        #print("X: "+str((b*(b*x0-a*y0)-a*c)/(a**2+b**2)))

        #print("Y: "+str((a*(-b*x0+a*y0)-b*c)/(a**2+b**2)))

        return abs((a*x0)+(b*y0)+c)/((a**2+b**2)**(0.5))


    # Given an image, a point (x,y), and a width/height,
    # Will return a "cropped image" of the same dimensions
    # where only a box whose upper left corner starts at (x,y) 
    # and a corresponding dimentions of width/height will correspond
    # to visible image. The rest of the image will be black.
    def CropVisible(self, image, x, y, width, height):
        
        # first, make 0 filled array of the same shape as the original image
        rect_image = zeros( (len(image),len(image[0]),len(image[0][0])), uint8 )
        # move each pixel that fits the (x,y) and width height criteria from image to the empty
        # array
        rect_image[y:y+height,x:x+width, 0:3:1] = image[y:y+height,x:x+width, 0:3:1]
        return rect_image 


    def DetectFaces(self,image,faceLength = 190):
        cascPath = expanduser("~")+"/drone_workspace/src/ardrone_lab/src/resources/haarcascade_frontalface_default.xml"

        # Create the haar cascade
        faceCascade = cv2.CascadeClassifier(cascPath)

        # Convert image to gray
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(30, 30),
        flags = cv2.CASCADE_SCALE_IMAGE)

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            distance = ( (685.2387*faceLength)/h)-223.3983
            cx = (2*x + w)/2
            cy = (2*y + w)/2
            return distance,(cx,cy)
        return None, None

    def DetectOrange(self, img, h=4):
        r,c,_ = img.shape
        segment = zeros((r,c,3), uint8)
        binary = zeros((r,c,1), uint8)
        for i in range(0,r,h):
            for j in range(0,c,h):
                if img[i,j][0]+20 < img[i,j][2] and img[i,j][1]+20 < img[i,j][2]:
                    for xi in range(0,h):
                        for yi in range(0,h):
                            segment[i+xi,j+yi] = img[i+xi,j+yi]
                            binary[i+xi,j+yi] = 1

        return segment,binary

