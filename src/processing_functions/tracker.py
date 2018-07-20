
import numpy as np
reload(np)
class Tracker(object):
    def __init__(self,roll = 0,pitch = 0,yaw = 0,translation = [0,0,0]):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.translation  = translation
        self.updateTransforms()

    def reset(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.translation = [0,0,0]
        self.updateTransforms()

    def update(self,roll,pitch,yaw,translation = [0,0,0]):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.translation = translation
        self.updateTransforms()

    #updates the internal transform matrices to get a point from body2World or world2Body
    def updateTransforms(self):
        translation = np.asarray(self.translation)
        self.world2BodyT,self.body2WorldT = self.world2BodyTrans(self.yaw,translation)
        self.world2CameraT,self.camera2WorldT = self.world2CameraTrans(self.roll,self.pitch,self.yaw,translation)
        self.body2CameraT,self.camera2BodyT = self.body2CameraTrans(self.roll,self.pitch)
    
#creates matrix for rotation around y axis by angle theta in degrees 
    def pitchM(self,angle):
        R = np.identity(3)
        theta = np.deg2rad(angle)
        R[1,1] = np.cos(theta)
        R[2,1] = np.sin(theta)
        R[1,2] = -np.sin(theta)
        R[2,2] = np.cos(theta)
        return R

    #creates matrix for rotation around x axis by angle theta in degrees 
    def rollM(self,angle):
        R = np.identity(3)
        theta = np.deg2rad(angle)
        R[0,0] = np.cos(theta)
        R[0,2] = np.sin(theta)
        R[2,0] = -np.sin(theta)
        R[2,2] = np.cos(theta)
        return R

    #creates matrix for rotation around z axis by angle theta in degrees 
    def yawM(self,angle):
        R = np.identity(3)
        theta = np.deg2rad(angle)
        R[0,0] = np.cos(theta)
        R[0,1] = -np.sin(theta)
        R[1,0] = np.sin(theta)
        R[1,1] = np.cos(theta)
        return R

    #takes the position of the drone relative to the world frame (translation) and the euler angles of the drone relative to world frame (roll,pitch,yaw) and returns a 4x4 matrix that will take a point in drone frame and return it as a point in the world frame.
    def camera2BodyTrans(self,roll,pitch,yaw = 0,translation=[0,0,0]):
        T = np.identity(4)
        T[:3,:3] = np.dot(self.yawM(yaw),np.dot(self.pitchM(pitch),self.rollM(roll)))
        T[:3,3] = np.asarray(translation)
        return T

    def body2CameraTrans(self,roll,pitch,yaw = 0,translation=[0,0,0]):
        T = np.identity(4)
        Tp = self.camera2BodyTrans(self,roll,pitch)
        T = np.linalg.inv(Tp)
        return T,Tp

    def body2WorldTrans(self,yaw,translation):
        T = np.identity(4)
        T[:3,:3] = self.yawM(yaw)
        T[:3,3] = translation
        return T

#takes position of the drone relative to the world frame (translation) and euler angles of the drone relative to world frame (roll,pitch,yaw) and returns a 4x4 matrix that will take a point in world frame and return it as a point in the world frame, also returns the body to world transformation as second return value.		
    def world2BodyTrans(self,yaw,translation):
        Tp = self.body2WorldTrans(yaw,translation)
        T = np.linalg.inv(Tp)
        return T,Tp

    def camera2WorldTrans(self,roll,pitch,yaw,translation):
        T = np.identity(4)
        T[:3,:3] = np.dot(self.yawM(yaw),np.dot(self.pitchM(pitch),self.rollM(roll)))
        T[:3,3] = np.asarray(translation)
        return T

    def world2CameraTrans(self,roll,pitch,yaw,translation):
        Tp = self.camera2WorldTrans(roll,pitch,yaw,translation)
        T = np.linalg.inv(Tp)
        return T,Tp

    #takes in a 3D-vector representing a 3D coordinate in the drone frame and converts it to the world frame
    def body2World(self,bodyVector):
        v = np.ones([4,1])
        v[:3,0] = np.asarray(bodyVector)
        return np.dot(self.body2WorldT,v)[:3]

    def world2Body(self,worldVector):
        v = np.ones([4,1])
        v[:3,0] = np.asarray(worldVector)
        return np.dot(self.world2BodyT,v)[:3]

    def camera2World(self,cameraVector):
        v = np.ones([4,1])
        v[:3,0] = np.asarray(cameraVector)
        return np.dot(self.camera2WorldT,v)[:3]

    def world2Camera(self,worldVector):
        v = np.ones([4,1])
        v[:3,0] = np.asarray(worldVector)
        return np.dot(self.world2CameraT,v)[:3]

    def transform(self,transform,vector):
        v = np.ones([4,1])
        v[:3,0] = np.asarray(vector)
        return np.dot(transform,vector)[:3]



