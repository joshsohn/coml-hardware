from enum import Enum
import quaternion
import numpy as np

class AttCmdClass:
    def __init__(self):
        self.q = np.quaternion(1, 0, 0, 0)
        self.w = np.zeros(3)
        self.F_W = np.zeros(3)

class ParametersClass:
    def __init__(self):
        self.mass = 0.0
        self.kp_xy = 0.0
        self.ki_xy = 0.0
        self.kd_xy = 0.0
        self.kp_z = 0.0
        self.ki_z = 0.0
        self.kd_z = 0.0
        self.maxPosErr_xy = 0.0
        self.maxPosErr_z = 0.0
        self.maxVelErr_xy = 0.0
        self.maxVelErr_z = 0.0

class StateClass:
    def __init__(self):
        self.t = -1
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.quaternion(1, 0, 0, 0)
        self.w = np.zeros(3)

class GoalClass:
    class Mode(Enum):
        POS_CTRL = 0
        VEL_CTRL = 1
        ACC_CTRL = 2

    def __init__(self):
        self.mode_xy = self.Mode.POS_CTRL
        self.mode_z = self.Mode.POS_CTRL
        self.t = -1
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.j = np.zeros(3)
        self.psi = 0
        self.dpsi = 0