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
    num_traj = 5
    T = 30
    num_knots = 6
    poly_orders = (9, 9, 9, 6, 6, 6)
    deriv_orders = (4, 4, 4, 2, 2, 2)
    min_step = jnp.array([-2., -2., 0, -jnp.pi/6, -jnp.pi/6, -jnp.pi/6])
    max_step = jnp.array([2., 2., 2., jnp.pi/6, jnp.pi/6, jnp.pi/6])
    min_knot = jnp.array([-jnp.inf, -jnp.inf, -jnp.inf, -jnp.pi/3, -jnp.pi/3, -jnp.pi/3])
    max_knot = jnp.array([jnp.inf, jnp.inf, jnp.inf, jnp.pi/3, jnp.pi/3, jnp.pi/3])

    key, *subkeys = jax.random.split(key, 1 + num_traj)
    subkeys = jnp.vstack(subkeys)
    in_axes = (0, None, None, None, None, None, None, None, None)
    t_knots, knots, coefs = jax.vmap(random_ragged_spline, in_axes)(
        subkeys, T, num_knots, poly_orders, deriv_orders,
        min_step, max_step, min_knot, max_knot
    )

    # Sample wind velocities from the training distribution
    w_min = 0.  # minimum wind velocity in inertial `x`-direction
    w_max = 6.  # maximum wind velocity in inertial `x`-direction
    a = 5.      # shape parameter `a` for beta distribution
    b = 9.      # shape parameter `b` for beta distribution
    key, subkey = jax.random.split(key, 2)
    w = w_min + (w_max - w_min)*jax.random.beta(subkey, a, b, (num_traj,))

    print(w)