print("  importing bar_fn, baz_fn from simon_pythonsubpackage1")
from .helpers import point_msg_to_array, vector_msg_to_array, quaternion_msg_to_quaternion, point_array_to_msg, vector_array_to_msg, quaternion_array_to_msg
from .simon_pythonsubpackage1 import bar_fn, baz_fn
from .structs import AttCmdClass, GoalClass, ParametersClass, StateClass
print("  imported bar_fn, baz_fn from simon_pythonsubpackage1")
import numpy as np
# import quaternion
import rospy
from snapstack_msgs.msg import State, Goal, AttitudeCommand, ControlLog

class OuterLoopROS:
    def __init__(self):
        # Initialize the ROS node with the default name 'my_node_name' (will be overwritten by launch file)
        rospy.init_node('my_node_name')   

        # Load ROS parameters
        self.p = self.load_parameters()

        self.state_ = StateClass()
        self.goal_ = GoalClass()

        # Subscribe and publish
        rospy.Subscriber('state', State, self.state_cb)
        rospy.Subscriber('goal', Goal, self.goal_cb)

        self.pub_att_cmd_ = rospy.Publisher('attcmd', AttitudeCommand, queue_size=1)
        self.pub_log_ = rospy.Publisher('log', ControlLog, queue_size=1)

        rospy.Timer(rospy.Duration(self.p.control_dt_), self.cntrl_cb)

        # Spin to keep the node alive and process callbacks
        rospy.spin()
    
    def load_parameters(self):
        p = ParametersClass()

        # Load parameters from the parameter server and assign loaded parameters to the Parameters instance
        p.control_dt_ = rospy.get_param("~control_dt", default=0.01)
        p.Tspinup_ = rospy.get_param("~spinup/time", default=1.0)
        p.spinup_thrust_gs_ = rospy.get_param("~spinup/thrust_gs", default=0.5)
        p.mass = rospy.get_param("~mass", default=1.0)
        p.kp_xy = rospy.get_param("~Kp/xy", default=1.0)
        p.ki_xy = rospy.get_param("~Ki/xy", default=0.0)
        p.kd_xy = rospy.get_param("~Kd/xy", default=0.0)
        p.kp_z = rospy.get_param("~Kp/z", default=1.0)
        p.ki_z = rospy.get_param("~Ki/z", default=0.0)
        p.kd_z = rospy.get_param("~Kd/z", default=0.0)
        p.maxPosErr_xy = rospy.get_param("~maxPosErr/xy", default=1.0)
        p.maxPosErr_z = rospy.get_param("~maxPosErr/z", default=1.0)
        p.maxVelErr_xy = rospy.get_param("~maxVelErr/xy", default=1.0)
        p.maxVelErr_z = rospy.get_param("~maxVelErr/z", default=1.0)
        p.alt_limit_ = rospy.get_param("~safety/alt_limit", default=6.0)

        return p

    # state callback function
    def state_cb(self, msg):
        statemsg_ = msg

        self.state_.t = msg.header.stamp.to_sec()
        self.state_.p = point_msg_to_array(msg.pos)
        self.state_.v = vector_msg_to_array(msg.vel)
        self.state_.q = quaternion_msg_to_quaternion(msg.quat)
        self.state_.w = vector_msg_to_array(msg.w)

    # goal callback function
    def goal_cb(self, msg):
        goalmsg_ = msg

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

        # Publilsh command via ROS
        attmsg = AttitudeCommand()
        attmsg.header.stamp = t_now
        attmsg.power = 0
        attmsg.q = quaternion_array_to_msg(cmd.q)
        attmsg.w = vector_array_to_msg(cmd.w)
        attmsg.F_W = vector_array_to_msg(cmd.F_W)

        self.pub_att_cmd_.publish(attmsg)

def main():
    OuterLoopROS()

if __name__ == '__main__':
    print(" executing main() in foo.py")
    bar_fn()
    baz_fn()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
