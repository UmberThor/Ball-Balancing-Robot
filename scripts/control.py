# The control law: pixel error of the ball and its derivative in, plate tilt out.

import numpy as np

# PID CONSTANTS
KP = 36
KI = 36
KD = 20
KP = KP*0.00001
KI = KI*0.00001
KD = KD*0.00001

# LQR CONSTANTS
K_LQR = np.array([-3.755751e-04, -2.744274e-04, -7.829578e-05])
e_k1 = [0, 0]
e_k2 = [0, 0]
a_k1 = [0, 0]
a_k2 = [0, 0]

# RL CONSTANTS AND POLICY
ERR_SCALE = 160
U_MAX = 0.14
W = np.load("policy.npz")

# CONTROL INITIALIZATION
err_int = np.zeros(2)       # integral error
err_prev = np.zeros(2)      # previous error

def reset(err):
    # reacquisition.
    # - the previous error is reinitialized at the current error
    # - the integral error is set to 0: whatever it wound up to while the ball was off the plate has nothing to do with the new position
    # - the rl state is set to 0
    global err_int, err_prev, e_k1, e_k2, a_k1, a_k2
    err_int = np.zeros(2)
    err_prev = err
    e_k1 = err / ERR_SCALE
    e_k2 = err / ERR_SCALE
    a_k1 = [0, 0]
    a_k2 = [0, 0]


# for both pid and lqr, err_der is the velocity of the error estimated by kalman.py.
# without it the velocity is the finite difference of the error, as it has always been

def pid(err, dt, err_der=None):
    global err_int, err_prev
    err_int = err_int + err * dt
    if err_der is None:
        err_der = (err - err_prev) / dt
    u = KP * err + KI * err_int + KD * err_der
    err_prev = err
    return u

def lqr(err, dt, err_der=None):
    global err_int, err_prev
    err_int = err_int + err_prev * dt
    if err_der is None:
        err_der = (err - err_prev) / dt
    err_prev = err
    u = - (K_LQR[0] * err + K_LQR[1] * err_der + K_LQR[2] * err_int)
    return u

def rl(e_k):
    global e_k1, e_k2, a_k1, a_k2
    e_k = e_k / ERR_SCALE

    obs = np.array([e_k[0], e_k[1], e_k1[0], e_k1[1], e_k2[0], e_k2[1], a_k1[0], a_k1[1], a_k2[0], a_k2[1]])
    h = np.maximum(obs @ W["0.weight"].T + W["0.bias"], 0)
    h = np.maximum(h @ W["2.weight"].T + W["2.bias"], 0)
    a_k = np.tanh(h @ W["mu.weight"].T + W["mu.bias"])

    e_k2 = e_k1.copy()
    e_k1 = e_k.copy()
    a_k2 = a_k1.copy()
    a_k1 = a_k.copy()

    return a_k * U_MAX