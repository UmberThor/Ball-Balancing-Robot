# The state estimator: measured pixel error of the ball in, filtered error and its velocity out.
# Pure numpy, so this module can be run off the robot.

import numpy as np

# MODEL CONSTANTS, the same of linear_quadratic_control.m
G_ACC = 9.81                            # m/s^2
PX_PER_M = 50.0 / 0.02                  # the ball radius is 50 px in the frame and 0.02 m in reality
C = -(3/5) * G_ACC * PX_PER_M           # px/s^2 per unit of slope, hollow shell

# KALMAN CONSTANTS
SIGMA_A = 100.0                         # px/s^2, std of the acceleration the model does not explain (friction, servo lag, pushes)
R_MEAS = 1.0                            # px^2, variance of the measured error
P0_POS = R_MEAS                         # px^2, at reacquisition the position is as good as the measurement
P0_VEL = 200.0**2                       # (px/s)^2, at reacquisition the velocity is unknown

H = np.array([[1.0, 0.0]])              # only the error is measured, not its velocity

# KALMAN INITIALIZATION
# x_hat[i] = [e, e_dot] along axis i. The two axes have the same model and the same noise,
# and the covariance does not depend on the measurements, so a single P serves both
x_hat = np.zeros((2, 2))                # state estimate
P = np.diag([P0_POS, P0_VEL])           # covariance of the estimate


def reset(err):
    # reacquisition: the position is the measurement, the velocity is unknown
    global x_hat, P
    x_hat = np.column_stack([err, np.zeros(2)])
    P = np.diag([P0_POS, P0_VEL])


def update(err, u, dt):
    # err is the measured error, u the slope commanded at the previous frame
    global x_hat, P
    F = np.array([[1.0, dt],
                  [0.0, 1.0]])
    G = np.array([C*dt**2/2, C*dt])
    Q = SIGMA_A**2 * np.array([[dt**4/4, dt**3/2],
                               [dt**3/2, dt**2  ]])

    # predict
    x_pred = x_hat @ F.T + np.outer(u, G)
    P_pred = F @ P @ F.T + Q

    # update
    y = err - x_pred[:, 0]
    S = (H @ P_pred @ H.T)[0, 0] + R_MEAS
    K = P_pred @ H.T[:, 0] / S
    x_hat = x_pred + np.outer(y, K)
    P = (np.eye(2) - np.outer(K, H[0])) @ P_pred

    return x_hat[:, 0].copy(), x_hat[:, 1].copy()     # filtered error, its velocity
