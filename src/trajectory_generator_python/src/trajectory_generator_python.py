#!/usr/bin/env python3

import numpy as np
import rospy
import pickle
from functools import partial
import jax
import jax.numpy as jnp
from jax.experimental.ode import odeint
from utils import spline, random_ragged_spline
from snapstack_msgs.msg import State, Goal, QuadFlightMode, ControlLog

class TrajectoryGenerator:
    def __init__(self):
        # Initialize the ROS node with the default name 'my_node_name' (will be overwritten by launch file)
        rospy.init_node('my_node_name')

        # Subscribe and publish
        rospy.Subscriber('/globalflightmode', QuadFlightMode, self.mode_cb)
        rospy.Subscriber('state', State, self.state_cb)
 
        self.pub_log_ = rospy.Publisher('log',ControlLog, queue_size=1)
        self.pub_goal_ = rospy.Publisher('goal',Goal, queue_size=1)

        alt_ = rospy.get_param('~alt', default=None)
        if alt_ is None:
            rospy.logerr("Parameter 'alt' not found!")

        freq = rospy.get_param('~pub_freq', default=None)
        if freq is None:
            rospy.logerr("Parameter 'pub_freq' not found!")
        dt_ = 1.0 / freq

        rospy.Timer(rospy.Duration(dt_), self.pub_cb)

        # Spin to keep the node alive and process callbacks
        rospy.spin()
    
    def mode_cb(self, msg):
        return
    
    def state_cb(self, msg):
        return
    
    def pub_cb(self, event):
        return
    
    def generate_trajectory(self):
        # Seed random numbers
        seed = 0
        key = jax.random.PRNGKey(seed)

        # Generate smooth trajectories
        num_traj = 500
        T = 30
        num_knots = 6
        poly_orders = (9, 9, 6)
        deriv_orders = (4, 4, 2)
        min_step = jnp.array([-2., -2., -jnp.pi/6])
        max_step = jnp.array([2., 2., jnp.pi/6])
        min_knot = jnp.array([-jnp.inf, -jnp.inf, -jnp.pi/3])
        max_knot = jnp.array([jnp.inf, jnp.inf, jnp.pi/3])

        key, *subkeys = jax.random.split(key, 1 + num_traj)
        subkeys = jnp.vstack(subkeys)
        in_axes = (0, None, None, None, None, None, None, None, None)
        t_knots, knots, coefs = jax.vmap(random_ragged_spline, in_axes)(
            subkeys, T, num_knots, poly_orders, deriv_orders,
            min_step, max_step, min_knot, max_knot
        )
        # x_coefs, y_coefs, ϕ_coefs = coefs
        r_knots = jnp.dstack(knots) 
    
        # Construct spline reference trajectory
        def reference(t):
            x_coefs, y_coefs, ϕ_coefs = coefs
            x = spline(t, t_knots, x_coefs)
            y = spline(t, t_knots, y_coefs)
            ϕ = spline(t, t_knots, ϕ_coefs)
            ϕ = jnp.clip(ϕ, -jnp.pi/3, jnp.pi/3)
            r = jnp.array([x, y, ϕ])
            return r

        # Required derivatives of the reference trajectory
        def ref_derivatives(t):
            ref_vel = jax.jacfwd(reference)
            ref_acc = jax.jacfwd(ref_vel)
            r = reference(t)
            dr = ref_vel(t)
            ddr = ref_acc(t)
            return r, dr, ddr

def main():
    TrajectoryGenerator()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
