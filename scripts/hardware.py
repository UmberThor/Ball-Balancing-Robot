# Everything that talks to the Raspberry Pi: the three servos and the camera.

from picamera2 import Picamera2
import pigpio
import time

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

def home():
    set_angle([Q_HOME + OFFSETS[i] for i in range(3)])

# CAMERA SETUP
TARGET_FPS = 40                          # frame rate demanded of the sensor
FRAME_US = int(1e6 / TARGET_FPS)         # the frame duration it corresponds to, in us

def camera_info(picam2):
    # what the camera is actually doing, as opposed to what it was asked to do
    md = picam2.capture_metadata()
    exp, dur, gain = md.get("ExposureTime"), md.get("FrameDuration"), md.get("AnalogueGain")
    if exp is None or dur is None:
        print(f"camera: no timing metadata, keys available: {sorted(md)}")
        return
    line = (f"camera: exposure {exp/1000:5.1f} ms | "
            f"frame {dur/1000:5.1f} ms ({1e6/dur:4.1f} fps)")
    if gain is not None:
        line += f" | gain {gain:.2f}"
    print(line)

def start_camera(W, H):
    picam2 = Picamera2()        # start the frame capturing
    config = picam2.create_preview_configuration(
        main={"size": (W, H), "format": "RGB888"},
        # a frame can never be shorter than the exposure it contains, so pinning
        # the frame duration also caps the exposure: the auto exposure then has
        # to reach for analogue gain instead of time, and the loop runs at the
        # rate the sensor can deliver rather than the rate the light allows
        controls={"FrameDurationLimits": (FRAME_US, FRAME_US)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)             # let the auto exposure settle before reading it back
    camera_info(picam2)
    return picam2

def shut_down(picam2):
    picam2.stop()
    for pin in SERVO_PINS:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
