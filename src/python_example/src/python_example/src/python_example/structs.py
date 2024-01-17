from enum import Enum
# import quaternion
import numpy as np

class AttCmdClass:
    def __init__(self):
        self.q = np.array([0, 0, 0, 1]) # x, y, z, w
        self.w = np.zeros(3)
        self.F_W = np.zeros(3)

class ParametersClass:
    def __init__(self):
        self.control_dt_ = 0.0
        self.Tspinup_ = 0.0
        self.spinup_thrust_gs_ = 0.0
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
        self.alt_limit_ = 0.0

class StateClass:
    def __init__(self):
        self.t = -1
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([0, 0, 0, 1]) # x, y, z, w
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

class ControlLogClass:
    def __init__(self):
        self.p = np.zeros(3)
        self.p_ref = np.zeros(3)
        self.p_err = np.zeros(3)
        self.p_err_int = np.zeros(3)
        self.v = np.zeros(3)
        self.v_ref = np.zeros(3)
        self.v_err = np.zeros(3)
        self.a_ff = np.zeros(3)
        self.a_fb = np.zeros(3)
        self.j_ff = np.zeros(3)
        self.j_fb = np.zeros(3)
        self.q = np.array([0, 0, 0, 1]) # x, y, z, w
        self.q_ref = np.array([0, 0, 0, 1]) # x, y, z, w
        self.w = np.zeros(3)
        self.w_ref = np.zeros(3)
        self.F_W = np.zeros(3)  # total desired force [N], expr in world frame
        self.mode_xy = GoalClass.Mode.POS_CTRL
        self.mode_z = GoalClass.Mode.POS_CTRL