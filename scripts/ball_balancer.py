import cv2
import numpy as np
import time
import argparse

import hardware
import vision
import control
import kinematics

# GUI ON/OFF
# python3 ball_balancer.py to run without image
# python3 ball_balancer.py --gui to run with image
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="show the camera window")
args = parser.parse_args()
gui_on = args.gui

fps_smooth = 30.0           # variable to keep track of the fps

t_prev = time.monotonic()   # instant of actuation
had_ball = False            # flag to know wether the previous frame had the ball

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

            if not had_ball:    # reacquisition, in case the ball is repositioned for example after falling
                control.reset(np.array([center[0]-cx, center[1]-cy], dtype=float))
                t_prev = time.monotonic()
            had_ball = True

            # CONTROL ACTION
            err = np.array([center[0] - cx, center[1] - cy], dtype=float)

            now = time.monotonic()
            dt = max(now - t_prev, 1e-3)
            t_prev = now

            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0/dt)

            u = control.pid(err, dt)

            # INVERSE KINEMATICS, see the matlab
            try:
                q_raw = kinematics.solve(u, kinematics.H_NOM)
                if q_raw is not None:
                    q_deg = np.clip(q_raw, hardware.Q_MIN, hardware.Q_MAX)
                    q_cmd = [q_deg[i] + hardware.OFFSETS[i] for i in range(3)]
                    hardware.set_angle(q_cmd)
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
