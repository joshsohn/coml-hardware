import numpy as np
from .structs import AttCmdClass, ControlLogClass

class OuterLoop:
    def __init__(self, params):
        self.params_ = params
        self.reset()

    def reset(self):
        self.Ix_ = 0
        self.Iy_ = 0
        self.Iz_ = 0
        self.log_ = ControlLogClass()
        self.a_fb_last_ = np.zeros(3)
        self.j_fb_last_ = np.zeros(3)
        self.t_last_ = 0

    def compute_attitude_command(self, t, state, goal):
        dt = 1e-2 if self.t_last == 0 else t - self.t_last

        if dt > 0:
            self.t_last = t
        else:
            print("Warning: non-positive dt:", dt, "[s].")

        F_W = self.get_force(dt, state, goal)
        q_ref = self.get_attitude(state, goal, F_W)
        w_ref = self.get_rates(dt, state, goal, F_W, self.log_.a_fb, q_ref)

        cmd = AttCmdClass()
        cmd.q = q_ref
        cmd.w = w_ref
        cmd.F_W = F_W
