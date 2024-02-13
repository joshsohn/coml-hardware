"""
TODO description.

Author: Spencer M. Richards
        Autonomous Systems Lab (ASL), Stanford
        (GitHub: spenrich)
"""

if __name__ == "__main__":
    import pickle
    from functools import partial
    import jax
    import jax.numpy as jnp
    from jax.experimental.ode import odeint
    from utils import spline, random_ragged_spline

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

    # Sampled-time simulator
    @partial(jax.vmap, in_axes=(None, 0, 0))
    def simulate(ts, t_knots, coefs):
        """TODO: docstring."""
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

        # Simulation loop
        def loop(carry, input_slice):
            t_prev = carry
            t = input_slice

            r, dr, ddr = ref_derivatives(t)
            carry = (t)
            output_slice = (r, dr, ddr)
            return carry, output_slice

        # Initial conditions
        t0 = ts[0]
        r0, dr0, ddr0 = ref_derivatives(t0)
        
        # Run simulation loop
        carry = (t0)
        carry, output = jax.lax.scan(loop, carry, ts[1:])
        r, dr, ddr = output

        # Prepend initial conditions
        r = jnp.vstack((r0, r))
        dr = jnp.vstack((dr0, dr))
        ddr = jnp.vstack((ddr0, ddr))

        return r, dr, ddr

    # Simulate tracking for each `w`
    dt = 0.01
    t = jnp.arange(0, T + dt, dt)  # same times for each trajectory
    # print('t_knots outside: ', t_knots.shape)
    r, dr, ddr = simulate(t, t_knots, coefs)

    print('r: ', r.shape)
    print('dr: ', dr.shape)
    print('ddr: ', ddr.shape)
