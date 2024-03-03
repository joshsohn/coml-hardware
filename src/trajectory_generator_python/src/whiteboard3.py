if __name__ == "__main__":
    import pickle
    import numpy as np

    with open('training_data.pkl', 'rb') as file:
        raw = pickle.load(file)
    
    w_train = np.asarray(raw['w'])
    w_min, w_max = raw['w_min'], raw['w_max']
    a, b = raw['beta_params']


    for key, value in raw.items():
        print(f"{key}: {value}")

    error = raw['r'] - raw['q']
    print(f"error: {error}")

    # data = {
    #     'seed': self.seed, 'prng_key': self.key,
    #     't': self.t, 'q': self.q, 'dq': self.dq,
    #     'u': self.u, 'r': self.r, 'dr': self.dr,
    #     't_knots': self.t_knots, 'r_knots': self.r_knots,
    #     'w': self.w, 'w_min': self.w_min, 'w_max': self.w_max,
    #     'beta_params': (self.a, self.b),
    # }