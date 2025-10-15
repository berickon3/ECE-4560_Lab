import time
import mujoco
import numpy as np

def Rx(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def Ry(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])
    
def Rz(thetadeg):
    thetarad = np.deg2rad(thetadeg)
    c = np.cos(thetarad)
    s = np.sin(thetarad)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

def get_gw1(theta1_deg):
    displacement = (0.0388353, 0.0, 0.0624)
    rotation = Rz(180) @ Rx(180) @ Rz(theta1_deg)
    pose = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    return pose
