# Everything that talks to the Raspberry Pi: the three servos and the camera.

from picamera2 import Picamera2
import pigpio
import time

# ACTUATION SETUP
SERVO_PINS = [13, 18, 12]
OFFSETS = [0, -5, -5]
Q_HOME = 60                              # motor angle at the nominal pose
Q_RANGE = 15                             # travel allowed around the nominal pose
Q_MIN, Q_MAX = Q_HOME - Q_RANGE, Q_HOME + Q_RANGE

# CONNECT PIGPIO
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

def angle_to_pulse(angle):
    # 0..180 deg -> 500..2500 us
    return 500 + (angle / 180.0) * 2000

def set_angle(angles):
    for i in range(3):
        pi.set_servo_pulsewidth(SERVO_PINS[i], angle_to_pulse(angles[i]))

def home():
    set_angle([Q_HOME + OFFSETS[i] for i in range(3)])

# CAMERA SETUP
TARGET_FPS = 20                   # frame rate demanded of the sensor
FRAME_US = int(1e6 / TARGET_FPS)  # the frame duration it corresponds to, in us

def start_camera(W, H):
    picam2 = Picamera2()          # start the frame capturing
    config = picam2.create_preview_configuration(
        main={"size": (W, H), "format": "RGB888"},
        controls={"FrameDurationLimits": (FRAME_US, FRAME_US)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    return picam2

def shut_down(picam2):
    picam2.stop()
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
