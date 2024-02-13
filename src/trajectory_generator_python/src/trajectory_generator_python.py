#!/usr/bin/env python3

import numpy as np
import rospy
from functools import partial
import jax
import jax.numpy as jnp
import math
from utils import spline, random_ragged_spline
from geometry_msgs.msg import Pose, Quaternion
from snapstack_msgs.msg import State, Goal, QuadFlightMode, ControlLog

class TrajectoryGenerator:
    def __init__(self):
        # Initialize the ROS node with the default name 'my_node_name' (will be overwritten by launch file)
        rospy.init_node('my_node_name')

        alt_ = rospy.get_param('~alt', default=None)
        if alt_ is None:
            rospy.logerr("Parameter 'alt' not found!")

        freq = rospy.get_param('~pub_freq', default=None)
        if freq is None:
            rospy.logerr("Parameter 'pub_freq' not found!")
        dt_ = 1.0 / freq

        self.traj_goals_full = self.generate_trajectory()
        self.index_msgs_full = []  # Initialize as needed

        # Subscribe and publish
        rospy.Subscriber('/globalflightmode', QuadFlightMode, self.mode_cb)
        rospy.Subscriber('state', State, self.state_cb)
 
        self.pub_timer_ = rospy.Timer(rospy.Duration(dt_), self.pub_cb)
        self.pub_goal_ = rospy.Publisher('goal',Goal, queue_size=1)

        rospy.sleep(1.0)  # To ensure that the state has been received

        self.flight_mode_ = "GROUND"
        self.pose_ = Pose()
        self.goal_ = Goal()

        self.reset_goal()
        self.goal_.p.x = self.pose_.position.x
        self.goal_.p.y = self.pose_.position.y
        self.goal_.p.z = self.pose_.position.z

        # Spin to keep the node alive and process callbacks
        rospy.spin()

        rospy.loginfo("Successfully launched trajectory generator node.")
    
    def mode_cb(self, msg):
        return
    
    def state_cb(self, msg):
        self.pose_.position.x = msg.pos.x
        self.pose_.position.y = msg.pos.y
        self.pose_.position.z = msg.pos.z
        self.pose_.orientation = msg.quat
    
    def pub_cb(self, event):
        return
    
    def generate_trajectory(self):
        start_point = np.array([0, 0, 1])
        end_point = np.array([1, 1, 1])

        # Define total time duration for the trajectory
        total_time = 1.0  # seconds

        # Calculate constant velocity
        velocity = (end_point - start_point) / total_time

        # Create time array
        time_array = np.linspace(0, total_time, num=3000)  # 3000 timesteps

        # Calculate position, velocity, and acceleration matrices
        r = np.outer(time_array, velocity) + np.tile(start_point, (len(time_array), 1))
        dr = np.tile(velocity, (len(time_array), 1))
        ddr = np.zeros_like(r)

        goals = []

        for r_i, dr_i, ddr_i in zip(r, dr, ddr):
            goals.append(self.create_goal(r_i, dr_i, ddr_i))
        
        print(goals)
        return goals

    def create_goal(self, r_i, dr_i, ddr_i):
        goal = Goal()
        goal.header.frame_id = "world"
        goal.p.x   = r_i[0]
        goal.p.y   = r_i[1]
        goal.p.z   = r_i[2]
        goal.v.x   = dr_i[0]
        goal.v.y   = dr_i[1]
        goal.v.z   = dr_i[2]
        goal.a.x = ddr_i[0]
        goal.a.y = ddr_i[1]
        goal.a.z = ddr_i[2]
        goal.power = True

        # FIXME: JERK, PSI, DPSI ARE INCORRECT!!!
        goal.j.x  = 0
        goal.j.y  = 0
        goal.j.z  = 0
        goal.psi = 0
        goal.dpsi = 0

        return goal

    def reset_goal(self):
        # Creating a new goal message should already set this correctly, but just in case
        # Exception: yaw would be 0 instead of current yaw
        self.goal_.p.x, self.goal_.p.y, self.goal_.p.z = 0, 0, 0
        self.goal_.v.x, self.goal_.v.y, self.goal_.v.z = 0, 0, 0
        self.goal_.a.x, self.goal_.a.y, self.goal_.a.z = 0, 0, 0
        self.goal_.j.x, self.goal_.j.y, self.goal_.j.z = 0, 0, 0
        #self.goal_.s.x, self.goal_.s.y, self.goal_.s.z = 0, 0, 0
        self.goal_.psi = self.quat2yaw(self.pose_.orientation)
        self.goal_.dpsi = 0
        # self.goal_.power = False
        # reset_xy_int and  reset_z_int are not used
        self.goal_.mode_xy = Goal.MODE_POSITION_CONTROL
        self.goal_.mode_z = Goal.MODE_POSITION_CONTROL
    
    def quat2yaw(self, q) -> float:
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z))
        return yaw

def main():
    TrajectoryGenerator()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
