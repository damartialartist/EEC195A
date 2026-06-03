# Lane Following Software
# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung

import csi
import time
import math
import pyb

from micropython import const
from pyb import Pin, Timer
from ulab import numpy as np

# --- Speed Constants ---
maxThrottle = 65
minThrottle = 40
blindThrottle = 40

# --- Walk the Dog ---
# maxThrottle = 30
# minThrottle = 18
# blindThrottle = 18


# ==========================================
# CLASSES
# ==========================================
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

        # PWM: DC Motor (Timer 2, 5kHz)
        self.timDC = Timer(2, freq=5000)
        self.pwmDCPos = self.timDC.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width=0)

        # PWM: Servo Motor (Timer 4, 100Hz)
        self.timServo = Timer(4, freq=100)
        self.pwmServo = self.timServo.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

        # INA / INB Directional Pins
        self.ina = Pin("P1", Pin.OUT_PP)
        self.inb = Pin("P2", Pin.OUT_PP)

        self.ina.value(0)
        self.inb.value(0)

    def Steer(self, mode, percentage=100):
        '''
        Drive the Servo Motor at a given percentage and direction, with center being 0%
        '''
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
        '''
        Drive the DC Motor at a given percentage of speed in a specified direction
        '''
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
csi0.auto_gain(False, gain_db=19.5)
csi0.auto_whitebal(False)
img = csi0.snapshot()


# ==========================================
# CAR INITIALIZATION
# ==========================================
car = Car()
car.Throttle(car.BRAKE)
car.Steer(car.STRAIGHT)
pyb.delay(const(1 * 10**3))


# ==========================================
# CONSTANTS & VISION PIPELINE SETTINGS
# ==========================================
WHITE_THRESH = [(140, 255)]
ORANGE_THRESH = [(30, 80, 15, 127, 15, 127)]

TRACKING_ROI = (0, 0, 80, 60)

clock = time.clock()


# ==========================================
# FUNCTIONS
# ==========================================
def find2Blobs(img: csi.image, ROI, lastXL, lastXR) -> None | list:
    '''
    Find the two largest blobs in an ROI, determining left and right.
    Use memory of previous blob locations if only one blob is in the ROI.
    '''

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
    '''
    Approximate curvature from lines on screen
    '''

    xvals = []
    yvals = []
    a, b, c = 0.0, 0.0, 0.0

    last_xL = 0         # leftmosts init value
    last_xR = 80        # rightmost init value
    TRACK_WIDTH = 60    # pixels approximately
    TRACK_HALF = TRACK_WIDTH / 2

    # Grab coordinates of in subROIs of the lines
    for i in reversed(range(12)):   # iterate from bottom of screen to top
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

    # Compute curvature constant from collected points
    xvalsarr = np.array(xvals)
    yvalsarr = np.array(yvals)
    if (len(xvalsarr) > 2):
        # Fit collected coordinates to a parabola
        a, b, c = np.polyfit(yvalsarr, xvalsarr, 2)
        x_pred = np.polyval((a, b, c), yvalsarr)
        res = abs(xvalsarr - x_pred)
        closest = np.argmin(res)
        closesty = yvalsarr[closest]
        kappa = 2 * a * (1 + ((2 * a * closesty) + b)**2)**(-1.5)
    else:
        # Not enough coordinate points to form a parabola
        kappa = 0

    return kappa, a, b, c


def standardize_line(l):
    '''
    Ensure the order returned of points is bottom to top of screen
    '''
    if l.y1() > l.y2():
        return l.x1(), l.y1(), l.x2(), l.y2()
    else:
        return l.x2(), l.y2(), l.x1(), l.y1()


def pid_ctrl(offset, angle, previous_error, previous_err_a, integral, kappa, dt):
    '''
    Return PD control value using offset as a proportional and angle as a derivative
    '''

    # Controller Coefficients
    kpo = 2
    kd = 0.3
    ki = 0
    kkap = 3

    icap = 0.5
    dt = dt if dt != 0 else 0.025

    # Normalize input errors
    e_off = offset / 40  # range -40 to 40

    # Eliminate offset error when small
    ignore_off = abs(e_off) <= 0.10

    # Compute integral component
    integral += e_off * dt

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
    '''
    Compute throttle percentage as a linear relationship dependant on absolute steering angle.
    minThrottle is used as a lower bbound and maxThrottle as the upper bound.
    '''
    absAngle = abs(steering_angle)
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
        .binary(thresholds=ORANGE_THRESH, zero=True)\
        .to_grayscale()\
        .binary(thresholds=WHITE_THRESH)

    img.draw_rectangle(TRACKING_ROI, color=127)

    line = img.get_regression([(255, 255)], robust=False, roi=TRACKING_ROI)

    if line:
        img.draw_line(line.line(), color=255)
        lx1, ly1, lx2, ly2 = standardize_line(line)

        last_seen_x = lx1
        is_recovering = False

        # Recovery Mode
        if off_track_flag == "RIGHT":
            # On Right Side of Track
            if lx1 > 40:
                # Recovery Successful, proceed to Control System
                off_track_flag = "NONE"
                last_known_side = "RIGHT"
            else:
                # Still recovering, hard steer left
                is_recovering = True
                car.Steer(car.LEFT, 100)
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        elif off_track_flag == "LEFT":
            # On Left Side of Track
            if lx1 < 40:
                # Recovery Successful, proceed to Control System
                off_track_flag = "NONE"
                last_known_side = "LEFT"
            else:
                # Still recovering, hard steer right
                is_recovering = True
                car.Steer(car.RIGHT, 100)
                car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

        # Control System Mode
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

            # Compute Curvature and Control Values
            kappa, a, b, c = get_curvature(img)
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
        # No lines detected, initiate recovery
        integral = 0

        if off_track_flag == "NONE":
            if last_seen_x < 15:
                off_track_flag = "RIGHT"
            elif last_seen_x > 65:
                off_track_flag = "LEFT"

        # Hard steer in the direction of the track based on where it left the screen
        if off_track_flag == "RIGHT":
            car.Steer(car.LEFT, 100)
        elif off_track_flag == "LEFT":
            car.Steer(car.RIGHT, 100)
        else:
            car.Steer(car.STRAIGHT)

        # Drive at blind speed
        car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)

    FPS = clock.fps()
    dt = 1 / FPS
