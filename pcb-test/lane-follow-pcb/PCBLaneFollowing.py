# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Integrated Lane-Keeping - Center-Seeking & Ultra-Stable Straightaways

import csi
import image
import time
import math
import pyb

from micropython import const
from pyb import Pin, Timer

# --- Speed Constants ---
maxThrottle = 40
minThrottle = 20
blindThrottle = 15

# ==========================================
# CLASSES
# ==========================================

class BlobMeasured():
    def __init__(self, xoff, deg):
        self.xoff = xoff
        self.deg = deg

class Car():
    def msToTicks(self, ms):
        return int((ms / 10) * 19200)

    def __init__(self):
        self.FULL_SPEED_REVERSE = self.RIGHT = self.msToTicks(1.1)
        self.STRAIGHT = self.BRAKE = self.msToTicks(1.5)
        self.FULL_SPEED_FORWARD = self.LEFT = self.msToTicks(1.9)
        self.OFF = 0

        self.currentMode = self.OFF
        self.CURR_STEER = self.STRAIGHT

        self.timDC = Timer(2, freq=15000)
        self.pwmDCPos = self.timDC.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width=0)

        self.timServo = Timer(4, freq=100)
        self.pwmServo = self.timServo.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

        self.ina = Pin("P1", Pin.OUT_PP)
        self.inb = Pin("P2", Pin.OUT_PP)

        self.ina.value(0)
        self.inb.value(0)

    def Steer(self, mode, percentage=100):
        percentage = max(min(percentage, 100), 0)
        modifier = self.msToTicks((0.4 * (1-percentage/100)))

        if (mode == self.RIGHT):
            width = mode - modifier
        elif (mode == self.LEFT):
            width = mode + modifier
        else:
            width = mode

        self.pwmServo.pulse_width(int(width))
        self.CURR_STEER = mode

    def Throttle(self, mode, percentage=100):
        percentage = max(min(percentage, 100), 0)

        if (mode == self.BRAKE):
            self.ina.value(0)
            self.inb.value(0)
            self.pwmDCPos.pulse_width_percent(0)
        elif (mode == self.FULL_SPEED_FORWARD):
            self.ina.value(1)
            self.inb.value(0)
            self.pwmDCPos.pulse_width_percent(int(percentage))
        elif (mode == self.FULL_SPEED_REVERSE):
            self.ina.value(0)
            self.inb.value(1)
            self.pwmDCPos.pulse_width_percent(int(percentage))

# ==========================================
# CAMERA SETUP & INITIALIZATION
# ==========================================
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QQQVGA)
csi0.auto_gain(False, gain_db=18.5)
csi0.auto_whitebal(False)
img = csi0.snapshot()

car = Car()
car.Throttle(car.BRAKE)
car.Steer(car.STRAIGHT)
pyb.delay(const(1 * 10**3))

# ==========================================
# CONSTANTS & VISION PIPELINE SETTINGS
# ==========================================

WHITE_THRESH = [(100, 255)] #100 for big lab room, 140 for smaller lab room
ORANGE_THRESH = [(30, 80, 15, 127, 15, 127)]

KERNEL_SIZE = 1
KERNEL = [ 0, -1,  0,
          -1,  5, -1,
           0, -1,  0]

# --- FULL VISION ROI ---
TRACKING_ROI = (0, 0, 80, 60)

clock = time.clock()

# ==========================================
# FUNCTIONS
# ==========================================

def standardize_line(l):
    if l.y1() > l.y2():
        return l.x1(), l.y1(), l.x2(), l.y2()
    else:
        return l.x2(), l.y2(), l.x1(), l.y1()

def pid_ctrl(offset, angle, previous_error, previous_err_a, integral, dt):
    # --- ULTRA-STABLE STRAIGHTAWAY TUNE ---
    kpo = 0.85  # Softened from 1.0 so it doesn't jump at tiny pixel shifts
    kpa = 1.2
    kd = 3.2    # Bumped from 2.5 to freeze the steering on straights
    ki = 0.1
    icap = 0.5
    dt = dt if dt != 0 else 0.025

    e_off = offset / 40
    e_ang = angle / 45

    integral += e_off * dt
    derivative = (e_off - previous_error) / dt + 0 * (e_ang - previous_err_a) / dt

    ignore_off = abs(e_off) <= 0.10
    ignore_ang = abs(e_ang) <= 0.10

    if (ignore_off and ignore_ang):
        prop = 0
    elif (ignore_off):
        prop = kpa * e_ang
    elif (ignore_ang):
        prop = kpo * e_off
    else:
        prop = kpo * e_off + kpa * e_ang

    if (e_off * previous_error < 0):
        integral *= 0.5

    if (integral > icap):
        integral = icap
    elif (integral < -icap):
        integral = -icap

    control = prop + ki * integral + kd * derivative
    return control, e_off, e_ang, integral

def ThrottleFromSteer(steering_angle):
    absAngle = abs(steering_angle)
    # Aggressive braking for corners
    if absAngle > 40:
        return minThrottle

    throttle = maxThrottle - (((maxThrottle - minThrottle) / 40) * absAngle)
    return max(throttle, minThrottle)

# ==========================================
# MAIN LOOP
# ==========================================

past_off = 0
past_ang = 0
integral = 0
dt = 2
throttle_percent = minThrottle

# Track Memory Variables
last_known_side = "RIGHT"
last_seen_x = 40
off_track_flag = "NONE"

while True:
    clock.tick()

    img = csi0.snapshot()\
        .lens_corr()\
        .binary(thresholds=ORANGE_THRESH,zero=True)\
        .to_grayscale()\
        .binary(thresholds=WHITE_THRESH)

    img.draw_rectangle(TRACKING_ROI, color=127)

    line = img.get_regression([(255, 255)], robust=True, roi=TRACKING_ROI)

    if line:
        img.draw_line(line.line(), color=255)
        lx1, ly1, lx2, ly2 = standardize_line(line)

        last_seen_x = lx1
        is_recovering = False

        # --- THE INSTANT RECOVERY OVERRIDE ---
        if off_track_flag == "RIGHT":
            if lx1 > 40:
                off_track_flag = "NONE"
                last_known_side = "RIGHT"
            else:
                is_recovering = True
                car.Steer(car.LEFT, 100)
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        elif off_track_flag == "LEFT":
            if lx1 < 40:
                off_track_flag = "NONE"
                last_known_side = "LEFT"
            else:
                is_recovering = True
                car.Steer(car.RIGHT, 100)
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        # --- NORMAL HUGGING LOGIC (CENTER-SEEKING) ---
        if not is_recovering:

            # 1. Identify what we are looking at (ignore the middle 10 pixels for memory)
            if lx1 < 35:
                last_known_side = "LEFT"
            elif lx1 > 45:
                last_known_side = "RIGHT"

            # 2. Calculate offset to keep the car in the middle of the lane
            if 35 <= lx1 <= 45:
                # We see BOTH lines (The Ghost Line). We are perfectly centered!
                # Target the dead center of the screen (X=40).
                offset = lx1 - 40

            elif last_known_side == "LEFT":
                # We drifted right and lost the right line. We only see the left line.
                # Push the left line back to X=15 to re-center the car.
                offset = lx1 - 15

            elif last_known_side == "RIGHT":
                # We drifted left and lost the left line. We only see the right line.
                # Push the right line back to X=65 to re-center the car.
                offset = lx1 - 65

            dx = lx2 - lx1
            dy = ly1 - ly2
            deflection_angle = math.degrees(math.atan2(dx, dy))

            control, past_off, past_ang, integral = pid_ctrl(offset, deflection_angle, past_off, past_ang, integral, dt)

            # E. Apply Steering & Throttle
            # --- WIDER DEADBAND ---
            # Ignores camera noise/micro-jitter on straightaways!
            if abs(control) < 0.12:
                car.Steer(car.STRAIGHT)
                car.Throttle(car.FULL_SPEED_FORWARD, maxThrottle)
            else:
                steer_percent = max(min(control, 1), -1) * 100
                if (steer_percent < 0):
                    car.Steer(car.LEFT, abs(steer_percent))
                else:
                    car.Steer(car.RIGHT, steer_percent)

                throttle_percent = ThrottleFromSteer(steer_percent)
                car.Throttle(car.FULL_SPEED_FORWARD, throttle_percent)

    else:
        # 5. NO LINES SEEN (EMERGENCY RECOVERY)
        integral = 0

        if off_track_flag == "NONE":
            if last_seen_x < 15:
                off_track_flag = "RIGHT"
            elif last_seen_x > 65:
                off_track_flag = "LEFT"

        if off_track_flag == "RIGHT":
            car.Steer(car.LEFT, 100)
        elif off_track_flag == "LEFT":
            car.Steer(car.RIGHT, 100)
        else:
            car.Steer(car.STRAIGHT)

        car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

    dt = 1 / clock.fps() if clock.fps() > 0 else 0.025
