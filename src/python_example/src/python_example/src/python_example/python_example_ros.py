print("  importing bar_fn, baz_fn from simon_pythonsubpackage1")
from .simon_pythonsubpackage1 import bar_fn, baz_fn
from .structs import AttCmdClass, GoalClass, ParametersClass, StateClass
print("  imported bar_fn, baz_fn from simon_pythonsubpackage1")
import numpy as np
# import quaternion
import rospy
from geometry_msgs.msg import Point, Quaternion, Vector3
from snapstack_msgs.msg import State, Goal, AttitudeCommand, ControlLog

# global variables
state_ = StateClass()
goal_ = GoalClass()

# convert from ROS message to array
def point_msg_to_array(point_msg):
    return np.array([point_msg.x, point_msg.y, point_msg.z])

def vector_msg_to_array(vector_msg):
    return np.array([vector_msg.x, vector_msg.y, vector_msg.z])

def quaternion_msg_to_quaternion(quaternion_msg):
    return np.array([quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w])

# convert from array to ROS message
def point_array_to_msg(point_array):
    return Point(x=point_array[0], y=point_array[1], z=point_array[2])

def vector_array_to_msg(vector_array):
    return Vector3(x=vector_array[0], y=vector_array[1], z=vector_array[2])

def quaternion_array_to_msg(quaternion_array):
    return Quaternion(x=quaternion_array[0], y=quaternion_array[1], z=quaternion_array[2], w=quaternion_array[3])

# state callback function
def state_cb(msg):
    statemsg_ = msg

    global state_
    state_.t = msg.header.stamp.to_sec()
    state_.p = point_msg_to_array(msg.pos)
    state_.v = vector_msg_to_array(msg.vel)
    state_.q = quaternion_msg_to_quaternion(msg.quat)
    state_.w = vector_msg_to_array(msg.w)

# goal callback function
def goal_cb(msg):
    goalmsg_ = msg

    global goal_
    goal_.t = msg.header.stamp.to_sec()
    goal_.p = point_msg_to_array(msg.p)
    goal_.v = vector_msg_to_array(msg.v)
    goal_.a = vector_msg_to_array(msg.a)
    goal_.j = vector_msg_to_array(msg.j)
    goal_.psi = msg.psi
    goal_.dpsi = msg.dpsi
    goal_.mode_xy = GoalClass.Mode(msg.mode_xy)
    goal_.mode_z = GoalClass.Mode(msg.mode_z)

# control callback function
def cntrl_cb(event):
    cmd = AttCmdClass()
    cmd.q = state_.q
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

    pub_att_cmd_.publish(attmsg)

def load_parameters():
    p = ParametersClass()

    # Load parameters from the parameter server
    global control_dt_, Tspinup_, spinup_thrust_gs_, alt_limit_
    control_dt_ = rospy.get_param("~control_dt", default=0.01)
    Tspinup_ = rospy.get_param("~spinup/time", default=1.0)
    spinup_thrust_gs_ = rospy.get_param("~spinup/thrust_gs", default=0.5)
    mass = rospy.get_param("~mass", default=1.0)
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
    alt_limit_ = rospy.get_param("~safety/alt_limit", default=6.0)

    # Assign loaded parameters to the Parameters instance
    p.mass = mass
    p.kp_xy = kp_xy
    p.ki_xy = ki_xy
    p.kd_xy = kd_xy
    p.kp_z = kp_z
    p.ki_z = ki_z
    p.kd_z = kd_z
    p.maxPosErr_xy = maxPosErr_xy
    p.maxPosErr_z = maxPosErr_z
    p.maxVelErr_xy = maxVelErr_xy
    p.maxVelErr_z = maxVelErr_z

    return p

def main():
    # Initialize the ROS node with the default name 'my_node_name' (will be overwritten by launch file)
    rospy.init_node('my_node_name')

    # Load ROS parameters
    p = load_parameters()

    # Subscribe and publish
    rospy.Subscriber('state', State, state_cb)
    rospy.Subscriber('goal', Goal, goal_cb)

    global pub_att_cmd_, pub_log_
    pub_att_cmd_ = rospy.Publisher('attcmd', AttitudeCommand, queue_size=1)
    pub_log_ = rospy.Publisher('log', ControlLog, queue_size=1)

    rospy.Timer(rospy.Duration(control_dt_), cntrl_cb)

    # Spin to keep the node alive and process callbacks
    rospy.spin()

if __name__ == '__main__':
    print(" executing main() in foo.py")
    bar_fn()
    baz_fn()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
