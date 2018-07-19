import numpy as np

def yaw(angle):
    R = np.identity(3)
    theta = np.deg2rad(angle)
    R[0,0] = np.cos(theta)
    R[0,1] = -np.sin(theta)
    R[1,0] = np.sin(theta)
    R[1,1] = np.cos(theta)
    return R

def body2World(angle,translation):
    R = yaw(angle)
    D = np.identity(4)
    D[:3,:3] = R
    D[:3,3] = translation
    return D
