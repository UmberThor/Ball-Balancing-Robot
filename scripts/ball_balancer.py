import cv2
import numpy as np
import time
import argparse

import hardware
import vision
import control
import kinematics
import kalman

# CONTROL MODE AND GUI ON/OFF
# exactly one control mode must be given: --pid, --lqr or --rl
# python3 ball_balancer.py --pid to run the PID without image
# python3 ball_balancer.py --pid --gui to run the PID with image
# python3 ball_balancer.py --lqr (--gui) to run the LQR
# python3 ball_balancer.py --lqr --kalman to feed the controller with the error and velocity estimated by kalman.py
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="show the camera window")
parser.add_argument("--kalman", action="store_true", help="estimate error and velocity with the Kalman filter instead of the finite difference")
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument("--pid", action="store_true", help="PID control")
mode.add_argument("--lqr", action="store_true", help="LQR control")
mode.add_argument("--rl", action="store_true", help="reinforcement learning control (not implemented yet)")
args = parser.parse_args()
gui_on = args.gui

fps_smooth = float(hardware.TARGET_FPS)    # variable to keep track of the fps

t_prev = time.monotonic()   # instant of actuation
had_ball = False            # flag to know wether the previous frame had the ball
u_prev = np.zeros(2)        # slope last applied to the plate, the input of the prediction of kalman.py

center = vision.center

picam2 = hardware.start_camera(vision.W, vision.H)

hardware.home()

try:
    count = 0       # holds the number of frames
    while True:
        count = count + 1
        frame = picam2.capture_array()

        ball, mask = vision.detect(frame)

        if ball is not None:
            cx, cy, r = ball

            err = np.array([center[0] - cx, center[1] - cy], dtype=float)

            if not had_ball:    # reacquisition, in case the ball is repositioned for example after falling
                control.reset(err)
                kalman.reset(err)
                t_prev = time.monotonic()
            had_ball = True

            # CONTROL ACTION
            now = time.monotonic()
            dt = max(now - t_prev, 1e-3)
            t_prev = now

            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0/dt)

            if args.kalman:
                err_ctrl, err_der = kalman.update(err, u_prev, dt)
            else:
                err_ctrl, err_der = err, None      # the controllers take the finite difference themselves

            if args.pid:
                u = control.pid(err_ctrl, dt, err_der)
            elif args.lqr:
                u = control.lqr(err_ctrl, dt, err_der)
            elif args.rl:
                print("UNDER CONSTRUCTION")
                exit()
                
            # INVERSE KINEMATICS, see the matlab
            try:
                q_raw = kinematics.solve(u, kinematics.H_NOM)
                if q_raw is not None:
                    q_deg = np.clip(q_raw, hardware.Q_MIN, hardware.Q_MAX)
                    q_cmd = [q_deg[i] + hardware.OFFSETS[i] for i in range(3)]
                    hardware.set_angle(q_cmd)
                    u_prev = u      # only a slope that reached the servos acts on the ball
                    if count % 5 == 0:
                        print(f"\rfps: {fps_smooth:5.1f} | err: [{err[0]:6.1f} {err[1]:6.1f}] | u: [{u[0]:+.3f} {u[1]:+.3f}] | q: [{q_cmd[0]:5.1f} {q_cmd[1]:5.1f} {q_cmd[2]:5.1f}]", end="", flush=True)
                else:
                    print("IK unreachable, skipping")
            except (ValueError, ZeroDivisionError, FloatingPointError):
                print("IK error, skipping")
        else:
            had_ball = False

        # show frame if requested. all the drawing lives in here, so that
        # nothing is drawn when there is no window to draw it in
        if gui_on:
            frame[mask == 0] = [0, 0, 0]

            if ball is not None:
                # display ball
                cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 3)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 0), 3)

                # display error and scaled version of control
                cv2.arrowedLine(frame, center, (int(cx), int(cy)), (255, 0, 0), 3, tipLength=0.15)
                s = 500
                tip = (int(center[0] + u[0]*s), int(center[1] + u[1]*s))
                cv2.arrowedLine(frame, center, tip, (0, 255, 255), 3, tipLength=0.15)

            frame_show = cv2.resize(frame, (648, 486))
            cv2.imshow("Camera", frame_show)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    print('Shut down')
    hardware.shut_down(picam2)
    cv2.destroyAllWindows()
