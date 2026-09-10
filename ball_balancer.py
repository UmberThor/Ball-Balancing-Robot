from picamera2 import Picamera2
import cv2
import numpy as np
import time
import pigpio
import argparse

# GUI ON/OFF
# python3 ball_balancer.py to run without image
# python3 ball_balancer.py --gui to run with image
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="show the camera window")
args = parser.parse_args()
gui_on = args.gui

# ACTUATION SETUP
SERVO_PINS = [13, 18, 12]
OFFSETS = [0, -5, -5]
Q_HOME = 60                              # motor angle at the nominal pose (h = H_NOM, plate horizontal)
Q_RANGE = 15                             # travel allowed around the nominal pose
Q_MIN, Q_MAX = Q_HOME - Q_RANGE, Q_HOME + Q_RANGE

# CONNECT PIGPIO (that uses DMA software pwm instead of Python software pwm)
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

def angle_to_pulse(angle):
    # 0..180 deg -> 500..2500 us, matching the old 2.5..12.5% duty at 50 Hz
    return 500 + (angle / 180.0) * 2000

def set_angle(angles):
    for i in range(3):
        pi.set_servo_pulsewidth(SERVO_PINS[i], angle_to_pulse(angles[i]))

# CONTROL CONSTANTS
KP = 0.00015
KI = 0.00025
KD = 0.00008

# DETECTION CONSTANTS
R_BALL_PX = 200.0                        # ball radius in pixels -> TO BE UPDATED!!
AREA_MIN = 0.10 * np.pi * R_BALL_PX**2   # a blob smaller then this cannot be the ball
FILL_MIN = 0.45                          # the contour detected as the ball should fill at least 45% of its minimum enclosing circle

# INVERSE KINEMATICS CONSTANTS
R_base = 28.65                           # radius of the base
R_plane = 84.00                          # radius of the plate
L1 = 80.0                                # length of link 1, the upper one (Pi -> Bi)
L2 = 80.0                                # length of link 2, the lower one (Mi -> Pi)
H_NOM = 120.0                            # height of the plate center at the nominal pose
base_c = np.array([0.0, 0.0, 0.0])       # coordinates of the base center
M1 = np.array([R_base*np.cos(np.pi),     R_base*np.sin(np.pi),     0.0]) # coordinates of motor 1
M2 = np.array([R_base*np.cos(5*np.pi/3), R_base*np.sin(5*np.pi/3), 0.0]) # coordinates of motor 2
M3 = np.array([R_base*np.cos(np.pi/3),   R_base*np.sin(np.pi/3),   0.0]) # coordinates of motor 3

fps_smooth = 30.0       # variable to keep track of the fps

# solve_P: given the coordinates of the motors (M) and of the ball joints (B),
# and the scalar factor that determines on which plane the pin joint must lie on (E),
# find the coordinates of the three pin joints
def solve_P(M, B, E, sign):
    A = (L1**2 - L2**2 + (M[0]**2 - B[0]**2 + M[1]**2 - B[1]**2 + M[2]**2 - B[2]**2))/(2*(M[2] - B[2]))
    Bc = (B[0] - M[0])/(M[2] - B[2])
    Cc = (B[1] - M[1])/(M[2] - B[2])
    a_coef = 1 + E**2 + (Bc + Cc*E)**2
    b_coef = -2*B[0] - 2*E*B[1] + 2*(Bc + Cc*E)*(A - B[2])
    c_coef = B[0]**2 + B[1]**2 + (A - B[2])**2 - L1**2
    Px = (-b_coef + sign*np.sqrt(b_coef**2 - 4*a_coef*c_coef))/(2*a_coef)
    return np.array([Px, E*Px, A + (Bc + Cc*E)*Px])


def find_ball(cnts):
    c = max(cnts, key=cv2.contourArea)          # find the contour of max area
    area = cv2.contourArea(c)                   # area of the contour
    if area < AREA_MIN:                         # first gate: the contour area must be >= 10% of a circle of radius R_BALL_PX
        return None
    (x, y), r = cv2.minEnclosingCircle(c)       # min eclosing circle of the candidate contour
    if area / (np.pi * r * r) < FILL_MIN:       # the candidate contour must be filled at >= 45% by the ball
        return None
    return x, y, r                              # center and radius of the minimum enclosing circle

# CONTROL INITIALIZATION
err_int = np.zeros(2)       # integral error
err_prev = np.zeros(2)      # previous error
t_prev = time.monotonic()   # instant of actuation

had_ball = False            # flag to know wether the previous frame had the ball

# IMAGE PROCESSING CONFIGURATION
W, H = 640, 480             # size of the camera frame
center = (W // 2, H // 2)

picam2 = Picamera2()        # start the frame capturing
config = picam2.create_preview_configuration(
    main={"size": (W, H), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

set_angle([Q_HOME + OFFSETS[i] for i in range(3)])

try:
    count = 0       # holds the number of frames
    while True:
        count = count + 1
        frame = picam2.capture_array()

        # IMAGE PROCESSING
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                                                                    # converts to hsv
        mask = (cv2.inRange(hsv, (148, 50, 50), (180, 255, 255)) | cv2.inRange(hsv, (0,   50, 50), (6,   255, 255)))    # keeps only pinkish pixels

        # defines two elliptical kernels and cleans the image externally of the ball (open)
        # and internally (close), so basically only the ball remains
        k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
        frame[mask == 0] = [0, 0, 0]

        # mask is a binary image, so we can pass it to findContours() that finds the contour of mask
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball = find_ball(cnts) if cnts else None

        if ball is not None:
            cx, cy, r = ball

            # display ball
            cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 3)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 0), 3)

            if not had_ball:    # reacquisition, in case the ball is repositioned for example after falling
                err_prev = np.array([center[0]-cx, center[1]-cy], dtype=float)
                t_prev = time.monotonic()
            had_ball = True

            # CONTROL ACTION
            err = np.array([center[0] - cx, center[1] - cy], dtype=float)

            now = time.monotonic()
            dt = max(now - t_prev, 1e-3)
            t_prev = now

            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0/dt)

            err_int = err_int + err * dt
            err_der = (err - err_prev) / dt
            u = KP * err + KI * err_int + KD * err_der
            err_prev = err

            # display error and scaled version of control
            cv2.arrowedLine(frame, center, (int(cx), int(cy)), (255, 0, 0), 3, tipLength=0.15)
            s = 500
            tip = (int(center[0] + u[0]*s), int(center[1] + u[1]*s))
            cv2.arrowedLine(frame, center, tip, (0, 255, 255), 3, tipLength=0.15)

            # INVERSE KINEMATICS, see the matlab
            try:
                n = np.array([u[0], u[1], 1.0])
                n = n / np.linalg.norm(n)
                h = H_NOM
                B1 = np.array([
                    -n[2]*R_plane/np.sqrt(n[0]**2 + n[2]**2),
                    0.0,
                    h + n[0]*R_plane/np.sqrt(n[0]**2 + n[2]**2)
                ])
                B2 = np.array([
                    n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2),
                    -np.sqrt(3)*n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2),
                    h + R_plane*(n[1]*np.sqrt(3) - n[0])/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2)
                ])
                B3 = np.array([
                    n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2),
                    np.sqrt(3)*n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2),
                    h - R_plane*(n[1]*np.sqrt(3) + n[0])/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2)
                ])
                dist1 = np.linalg.norm(B1-M1)
                dist2 = np.linalg.norm(B2-M2)
                dist3 = np.linalg.norm(B3-M3)
                ok = (abs(L1-L2) < dist1 < L1+L2 and abs(L1-L2) < dist2 < L1+L2 and abs(L1-L2) < dist3 < L1+L2)
                if ok:
                    P1 = solve_P(M1, B1, 0.0, -1)
                    P2 = solve_P(M2, B2, -np.sqrt(3), +1)
                    P3 = solve_P(M3, B3, np.sqrt(3), +1)
                    q1 = np.arctan(np.sqrt((P1[1]-M1[1])**2 + (P1[0]-M1[0])**2)/abs(P1[2]-M1[2]))
                    q2 = np.arctan(np.sqrt((P2[1]-M2[1])**2 + (P2[0]-M2[0])**2)/abs(P2[2]-M2[2]))
                    q3 = np.arctan(np.sqrt((P3[1]-M3[1])**2 + (P3[0]-M3[0])**2)/abs(P3[2]-M3[2]))
                    q_deg = np.clip(np.degrees([q1, q2, q3]), Q_MIN, Q_MAX)
                    q_cmd = [q_deg[i] + OFFSETS[i] for i in range(3)]
                    set_angle(q_cmd)
                    if count % 5 == 0:
                        print(f"\rfps: {fps_smooth:5.1f} | err: [{err[0]:6.1f} {err[1]:6.1f}] | u: [{u[0]:+.3f} {u[1]:+.3f}] | q: [{q_cmd[0]:5.1f} {q_cmd[1]:5.1f} {q_cmd[2]:5.1f}]", end="", flush=True)
                else:
                    print("IK unreachable, skipping")
            except (ValueError, ZeroDivisionError, FloatingPointError):
                print("IK error, skipping")
        else:
            had_ball = False

        # show frame if requested
        if gui_on:
            frame_show = cv2.resize(frame, (648, 486))
            cv2.imshow("Camera", frame_show)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
finally:
    print('Shut down')
    picam2.stop()
    cv2.destroyAllWindows()
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()