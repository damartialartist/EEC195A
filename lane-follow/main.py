# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Integrated Lane-Keeping - Aggressive Braking & Instant Recovery Override

import csi
# import image
import time
import math
import pyb

from micropython import const
from pyb import Pin, Timer
from ulab import numpy as np

# --- Speed Constants ---
maxThrottle = 30
minThrottle = 18
blindThrottle = 18

# --- walk the dog ---
# maxThrottle = 20
# minThrottle = 15
# blindThrottle = 15

kickstartThrottle = 70
KICKSTART_TIME = 0.5

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
        # Modes
        self.FULL_SPEED_REVERSE = self.RIGHT = self.msToTicks(1.1)
        self.STRAIGHT = self.BRAKE = self.msToTicks(1.5)
        self.FULL_SPEED_FORWARD = self.LEFT = self.msToTicks(1.9)
        self.OFF = 0

        self.currentMode = self.OFF
        self.CURR_STEER = self.STRAIGHT

        # PWM: DC Motor (VNH Driver on Timer 2, 10kHz)
        self.timDC = Timer(2, freq=5000)
        self.pwmDCPos = self.timDC.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width=0)

        # PWM: Servo Motor (Standard RC on Timer 4, 100Hz)
        self.timServo = Timer(4, freq=100)
        self.pwmServo = self.timServo.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

        # INA / INB Directional Pins
        self.ina = Pin("P1", Pin.OUT_PP)
        self.inb = Pin("P2", Pin.OUT_PP)

        self.ina.value(0)
        self.inb.value(0)

    def Steer(self, mode, percentage=100):
        percentage = max(min(percentage, 100), 0)
        modifier = self.msToTicks((0.4 * (1-percentage/100)))

        if (mode == self.RIGHT):
            width = mode + modifier
        elif (mode == self.LEFT):
            width = mode - modifier
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

WHITE_THRESH = [(140, 255)]
ORANGE_THRESH = [(30, 80, 15, 127, 15, 127)]

KERNEL_SIZE = 1
KERNEL = [0, -1,  0,
          -1,  5, -1,
          0, -1,  0]

TRACKING_ROI = (0, 0, 80, 60)

clock = time.clock()

# ==========================================
# FUNCTIONS
# ==========================================


def find2Blobs(img: csi.image, ROI, lastXL, lastXR) -> None | list:
    threshold_list = [(240, 255)]
    blob_array = img.find_blobs(
        threshold_list, pixels_threshold=5, area_threshold=5, merge=True, roi=ROI)

    blob1 = None
    blob2 = None
    blobleft = None
    blobright = None

    if blob_array:
        blob_array.sort(key=lambda b: b.pixels(), reverse=True)
        blob1 = blob_array[0]
        if len(blob_array) >= 2:
            blob2 = blob_array[1]
            cx1, cx2 = blob1.cx(), blob2.cx()
            if (cx1 < cx2):
                blobleft = blob1
                blobright = blob2
            else:
                blobleft = blob2
                blobright = blob1
        elif (len(blob_array) == 1):
            if (abs(blob1.cx() - lastXL) > abs(blob1.cx() - lastXR)):
                blobleft = None
                blobright = blob1
            else:
                blobleft = blob1
                blobright = None
    else:
        blobleft = None
        blobright = None
    return blobleft, blobright


def get_curvature(img: csi.image):
    xvals = []
    yvals = []
    a, b, c = 0.0, 0.0, 0.0
    TRACK_WIDTH = 60  # pixels approximately
    TRACK_HALF = TRACK_WIDTH / 2
    last_xL = 0     # leftmosts init value
    last_xR = 80    # rightmost init value
    for i in reversed(range(12)):
        cx = None
        cy = None
        subROI = (0, 5*i, 80, 5)
        blobL, blobR = find2Blobs(img, subROI, last_xL, last_xR)
        if (blobL and blobR):
            last_xL = blobL.cx()
            last_xR = blobR.cx()
            cx = 0.5 * (blobL.cx() + blobR.cx())
            cy = 0.5 * (blobL.cy() + blobR.cy())
        elif (blobL):
            last_xL = blobL.cx()
            cx = blobL.cx() + TRACK_HALF
            cy = blobL.cy()
        elif (blobR):
            last_xR = blobR.cx()
            cx = blobR.cx() - TRACK_HALF
            cy = blobR.cy()
        else:
            continue

        xvals.append(cx)
        yvals.append(cy)

    xvalsarr = np.array(xvals)
    yvalsarr = np.array(yvals)
    if (len(xvalsarr) > 2):
        a, b, c = np.polyfit(yvalsarr, xvalsarr, 2)
        x_pred = np.polyval((a, b, c), yvalsarr)
        res = abs(xvalsarr - x_pred)
        closest = np.argmin(res)
        closesty = yvalsarr[closest]
        # closesty = 20
        kappa = 2 * a * (1 + ((2 * a * closesty) + b)**2)**(-1.5)
    else:
        kappa = 0
    return kappa, a, b, c


def standardize_line(l):
    if l.y1() > l.y2():
        return l.x1(), l.y1(), l.x2(), l.y2()
    else:
        return l.x2(), l.y2(), l.x1(), l.y1()


def pid_ctrl(offset, angle, previous_error, previous_err_a, integral, kappa, dt):
    # goal: use angle as differential ctrl instead of another prop component
    # also dependent on curvature kappa

    # Controller Coefficients
    kpo = 1.5
    kd = 0.25
    ki = 0.1
    kkap = 2.5

    # kpo = 1.8
    # kd = 1.6
    # ki = 0.1
    # kkap = 2

    icap = 0.5
    dt = dt if dt != 0 else 0.025

    # Normalize input errors
    e_off = offset / 40  # range -40 to 40
    # Eliminate offset error when small
    ignore_off = abs(e_off) <= 0.10

    if (ignore_off):
        prop = 0
    else:
        prop = kpo * e_off

    if (e_off * previous_error < 0):
        integral *= 0.5

    # Max/Min cap of integral
    if (integral > icap):
        integral = icap
    elif (integral < -icap):
        integral = -icap

    control = prop + ki * integral + kd * math.sin(angle) + kkap * kappa
    return control, e_off, integral


def ThrottleFromSteer(steering_angle):
    absAngle = abs(steering_angle)
    # AGGRESSIVE BRAKING: If steering is more than 40%, slam on the brakes!
    # if absAngle > 40:
    #     return minThrottle
    # Otherwise, smoothly scale speed down from max
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

# add starting boost
# car.Throttle(car.FULL_SPEED_FORWARD, kickstartThrottle)
# time.sleep(KICKSTART_TIME)
# car.Throttle(car.FULL_SPEED_FORWARD, minThrottle)

while True:
    clock.tick()

    img = csi0.snapshot()\
        .lens_corr()\
        .binary(thresholds=ORANGE_THRESH, zero=True)\
        .to_grayscale()\
        .binary(thresholds=WHITE_THRESH)

    img.draw_rectangle(TRACKING_ROI, color=127)

    for i in range(12):
        img.draw_rectangle((0, 5*i, 80, 5), color=100)

    line = img.get_regression([(255, 255)], robust=False, roi=TRACKING_ROI)

    if line:
        img.draw_line(line.line(), color=255)
        lx1, ly1, lx2, ly2 = standardize_line(line)

        last_seen_x = lx1
        is_recovering = False

        # --- THE INSTANT RECOVERY OVERRIDE ---
        if off_track_flag == "RIGHT":
            if lx1 > 40:
                off_track_flag = "NONE"  # Successfully crossed back inside!
                last_known_side = "RIGHT"
            else:
                is_recovering = True
                car.Steer(car.LEFT, 100)  # Hard lock servo, ignore PID!
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        elif off_track_flag == "LEFT":
            if lx1 < 40:
                off_track_flag = "NONE"  # Successfully crossed back inside!
                last_known_side = "LEFT"
            else:
                is_recovering = True
                car.Steer(car.RIGHT, 100)  # Hard lock servo, ignore PID!
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        # --- NORMAL HUGGING LOGIC ---
        if not is_recovering:
            if lx1 < 25:
                last_known_side = "LEFT"
            elif lx1 > 55:
                last_known_side = "RIGHT"

            if last_known_side == "LEFT":
                offset = lx1 - 5
            else:
                offset = lx1 - 75

            dx = lx2 - lx1
            dy = ly1 - ly2
            deflection_angle = math.atan2(dx, dy)

            kappa, a, b, c = get_curvature(img)
            # kappa = 0
            control, past_off, integral = pid_ctrl(offset, deflection_angle, past_off, past_ang, integral, kappa, dt)

            if abs(control) < 0.08:
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

    FPS = clock.fps()
    dt = 1 / FPS
