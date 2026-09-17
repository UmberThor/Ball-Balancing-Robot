# The control law: pixel error of the ball in, plate tilt out.
# LQR and reinforcement learning go here too, next to the PID.

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

# CONTROL INITIALIZATION
err_int = np.zeros(2)       # integral error
err_prev = np.zeros(2)      # previous error


def reset(err):
    # reacquisition, in case the ball is repositioned for example after falling.
    # the integral is dropped too: whatever it wound up to while the ball was
    # off the plate has nothing to do with the new position
    global err_int, err_prev
    err_int = np.zeros(2)
    err_prev = err


# err_der is the velocity of the error estimated by kalman.py. Without it the
# velocity is the finite difference of the error, as it has always been
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
