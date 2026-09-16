# The control law: pixel error of the ball in, plate tilt out.
# LQR and reinforcement learning go here too, next to the PID.

import numpy as np

# CONTROL CONSTANTS
KP = 24
KI = 48
KD = 12
print(f'KP:{KP}\tKI:{KI}\t KD:{KD}')
KP = KP*0.00001
KI = KI*0.00001
KD = KD*0.00001

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


def pid(err, dt):
    global err_int, err_prev
    err_int = err_int + err * dt
    err_der = (err - err_prev) / dt
    u = KP * err + KI * err_int + KD * err_der
    err_prev = err
    return u
