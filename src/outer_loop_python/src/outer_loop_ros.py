#!/usr/bin/env python3.8

from helpers import point_msg_to_array, vector_msg_to_array, quaternion_msg_to_quaternion, point_array_to_msg, vector_array_to_msg, quaternion_array_to_msg
from outer_loop import OuterLoop
from structs import AttCmdClass, GoalClass, ModeClass, ParametersClass, StateClass
import numpy as np
import rospy
from snapstack_msgs.msg import State, Goal, AttitudeCommand, ControlLog

class OuterLoopROS:
    def __init__(self):
        # Initialize the ROS node with the default name 'my_node_name' (will be overwritten by launch file)
        rospy.init_node('my_node_name')   

        self.control_dt_ = 0.0
        self.Tspinup_ = 0.0
        self.spinup_thrust_gs_ = 0.0
        self.spinup_thrust_ = 0.0
        self.alt_limit_ = 0.0

        self.safety_kill_reason = ""
        self.mode_ = ModeClass.Preflight

        # Load ROS parameters
        self.p = self.load_parameters()

        self.state_ = StateClass()
        self.goal_ = GoalClass()

        self.statemsg_ = State()
        self.goalmsg_ = Goal()

        self.olcntrl_ = OuterLoop(self.p)

        # Subscribe and publish
        rospy.Subscriber('state', State, self.state_cb)
        rospy.Subscriber('goal', Goal, self.goal_cb)
 
        self.pub_att_cmd_ = rospy.Publisher('attcmd', AttitudeCommand, queue_size=1)
        self.pub_log_ = rospy.Publisher('log',ControlLog, queue_size=1)

        rospy.Timer(rospy.Duration(self.control_dt_), self.cntrl_cb)

        # Spin to keep the node alive and process callbacks
        rospy.spin()
    
    def load_parameters(self):
        p = ParametersClass()

        p.mass = rospy.get_param("~mass", default=1.0)

        # Load parameters from the parameter server and assign loaded parameters to the Parameters instance
        self.control_dt_ = rospy.get_param("~control_dt", default=0.01)
        self.Tspinup_ = rospy.get_param("~spinup/time", default=1.0)
        self.spinup_thrust_gs_ = rospy.get_param("~spinup/thrust_gs", default=0.5)
        self.alt_limit_ = rospy.get_param("~safety/alt_limit", default=6.0)

        # Make sure that spinup_thrust is safe
        if self.spinup_thrust_gs_ > 0.9:
            rospy.logwarn(f"You requested a spinup/thrust_gs of {self.spinup_thrust_gs_}."
                        f"\n\nThis value is used to calculate the idling throttle of the "
                        f"motors before takeoff and should be just enough to make them "
                        f"spin (getting out of the nonlinearity of motors turning on) "
                        f"but not enough to overcome the force of gravity, F = mg."
                        f"\n\nOverriding spinup/thrust_gs with a safer value, 0.5")
            self.spinup_thrust_gs_ = 0.5

        # Calculate thrust in [N] to use
        self.spinup_thrust_ = self.spinup_thrust_gs_ * p.mass * 9.81

        kp_xy = rospy.get_param("~Kp/xy", default=1.0)
        ki_xy = rospy.get_param("~Ki/xy", default=0.0)
        kd_xy = rospy.get_param("~Kd/xy", default=0.0)
        kp_z = rospy.get_param("~Kp/z", default=1.0)
        ki_z = rospy.get_param("~Ki/z", default=0.0)
        kd_z = rospy.get_param("~Kd/z", default=0.0)
        maxPosErr_xy = rospy.get_param("~maxPosErr/xy", default=1.0)
        maxPosErr_z = rospy.get_param("~maxPosErr/z", default=1.0)
        maxVelErr_xy = rospy.get_param("~maxVelErr/xy", default=1.0)
        maxVelErr_z = rospy.get_param("~maxVelErr/z", default=1.0)

        p.Kp = np.array([kp_xy, kp_xy, kp_z])
        p.Ki = np.array([ki_xy, ki_xy, ki_z])
        p.Kd = np.array([kd_xy, kd_xy, kd_z])
        p.maxPosErr = np.array([maxPosErr_xy, maxPosErr_xy, maxPosErr_z])
        p.maxVelErr = np.array([maxVelErr_xy, maxVelErr_xy, maxVelErr_z])
        
        return p

    # state callback function
    def state_cb(self, msg):
        self.statemsg_ = msg

        self.state_.t = msg.header.stamp.to_sec()
        self.state_.p = point_msg_to_array(msg.pos)
        self.state_.v = vector_msg_to_array(msg.vel)
        self.state_.q = quaternion_msg_to_quaternion(msg.quat)
        self.state_.w = vector_msg_to_array(msg.w)

    # goal callback function
    def goal_cb(self, msg):
        self.goalmsg_ = msg

        self.goal_.t = msg.header.stamp.to_sec()
        self.goal_.p = point_msg_to_array(msg.p)
        self.goal_.v = vector_msg_to_array(msg.v)
        self.goal_.a = vector_msg_to_array(msg.a)
        self.goal_.j = vector_msg_to_array(msg.j)
        self.goal_.psi = msg.psi
        self.goal_.dpsi = msg.dpsi
        self.goal_.mode_xy = GoalClass.Mode(msg.mode_xy)
        self.goal_.mode_z = GoalClass.Mode(msg.mode_z)

    # control callback function
    def cntrl_cb(self, event):
        cmd = AttCmdClass()
        cmd.q = self.state_.q
        cmd.w = [0.0, 0.0, 0.0]
        cmd.F_W = [0.0, 0.0, 0.0]

        t_now = rospy.Time.now()
        
        if t_now.is_zero():
            return

        # If high-level planner doesn't allow power, set mode to Preflight
        if not self.goalmsg_.power:
            self.mode_ = ModeClass.Preflight

        # Passthrough the current state and goal to keep log up to date
        self.olcntrl_.update_log(self.state_)

        # Flight Sequence State Machine
        if self.mode_ == ModeClass.Preflight:
            # print('preflight')
            if self.goalmsg_.power:
                self.mode_ = ModeClass.SpinningUp
                self.t_start = rospy.get_time()

            if not self.do_preflight_checks_pass():
                self.mode_ = ModeClass.Preflight

        elif self.mode_ == ModeClass.SpinningUp:
            # print('spinning up')
            if rospy.get_time() < self.t_start + self.Tspinup_:
                cmd.q = self.state_.q
                cmd.w = np.zeros(3)
                cmd.F_W = self.spinup_thrust_ * np.array([0.0, 0.0, 1.0])
            else:
                self.olcntrl_.reset()
                self.mode_ = ModeClass.Flying
                rospy.logwarn_throttle(0.5, "Spinning up motors.")

        elif self.mode_ == ModeClass.Flying:
            # print('flying')
            cmd = self.olcntrl_.compute_attitude_command(rospy.get_time(), self.state_, self.goal_)

            # Safety checks
            if not self.do_safety_checks_pass():
                self.mode_ = ModeClass.EmergencyStop

        elif self.mode_ == ModeClass.EmergencyStop:
            # print('emergency stop')
            cmd.q = self.state_.q
            cmd.w = np.zeros(3)
            cmd.F_W = np.zeros(3)
            rospy.logwarn_throttle(0.5, "Emergency stop.")

        # Publish command via ROS
        attmsg = AttitudeCommand()
        attmsg.header.stamp = t_now
        attmsg.power = 1
        attmsg.q = quaternion_array_to_msg(cmd.q)
        attmsg.w = vector_array_to_msg(cmd.w)
        attmsg.F_W = vector_array_to_msg(cmd.F_W)

        self.pub_att_cmd_.publish(attmsg)

    def do_preflight_checks_pass(self):
        return self.check_state()

    def check_state(self):
        if self.state_.t == -1:
            rospy.logwarn_throttle(0.5, "Preflight checks --- waiting on state data from autopilot. "
                                        "Is IMU calibration complete?")
            return False

        return True

    def do_safety_checks_pass(self):
        return self.check_altitude() and self.check_comms()

    def check_altitude(self):
        if self.state_.p[2] > self.alt_limit_:
            rospy.logerr(f"Safety --- Altitude check failed ({self.state_.p[2]} > {self.alt_limit_}).")
            self.safety_kill_reason += "ALT "
            return False

        return True

    def check_comms(self):
        if False:  # Replace with actual communication check
            self.safety_kill_reason += "COMM "

        return True

def main():
    OuterLoopROS()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass