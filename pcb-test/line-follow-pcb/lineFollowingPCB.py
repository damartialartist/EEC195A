# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Integrated Line-Following with Custom VNH Motor PCB

import csi
import image
import time
import math
import pyb

from micropython import const
from pyb import Pin, Timer

# --- Speed Constants ---
maxThrottle = 22
minThrottle = 18
blindThrottle = 18

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

        # PWM: DC Motor (VNH Driver on Timer 2, 15kHz)
        self.timDC = Timer(2, freq=15000)
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

        # Corrected steering math to prevent out-of-bounds PWM pulses
        if (mode == self.RIGHT):
            width = mode - modifier
        elif (mode == self.LEFT):
            width = mode + modifier
        else:
            width = mode

        self.pwmServo.pulse_width(int(width))
        self.CURR_STEER = mode

    def Throttle(self, mode, percentage=100):
        # Clamp percentage for safety
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
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.QQQVGA)
csi0.auto_gain(False, gain_db=18.5)
csi0.auto_whitebal(False)
img = csi0.snapshot()

# Car Initialization
car = Car()
car.Throttle(car.BRAKE)
car.Steer(car.STRAIGHT)
pyb.delay(const(1 * 10**3)) # 1 second delay to let things settle

# Constants & Vision
img_width = img.width()
img_middle_x = img_width // 2

thresholds = (200, 230)
ROIs = (0, 0, 160, 10, 0.5)  # x y w h (weight)
THRESHOLD = (190, 255)
BINARY_VISIBLE = True

clock = time.clock()

# ==========================================
# FUNCTIONS
# ==========================================

def findTopBlob(img: csi.image) -> None | image.Blob:
    threshold_list = [thresholds]
    blob_array = img.find_blobs(
        threshold_list, pixels_threshold=5, area_threshold=5, merge=True, roi=ROIs[0:4]
    )
    if blob_array:
        middle_blob = min(blob_array, key=lambda b: abs(b.cx() - img_middle_x))
    else:
        middle_blob = None
    return middle_blob

def drawTopBlob(img: csi.image, blob: [None | image.Blob]) -> None:
    if blob is None:
        return
    if blob.elongation() > 0.5:
        img.draw_edges(blob.min_corners(), color=0)
        img.draw_line(blob.major_axis_line(), color=0)
        img.draw_line(blob.minor_axis_line(), color=0)

    img.draw_rectangle(blob.rect(), color=127)
    img.draw_cross(blob.cx(), blob.cy(), color=127)
    img.draw_keypoints(
        [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))],
        size=40, color=127,
    )
    return

def findBlobErr(blob):
    b0 = blob
    if b0 is None:
        return None
    deg = math.degrees(math.atan2(b0.cy() - 80, b0.cx() - 80)) - 90
    xoffb0 = b0.cx() - img_middle_x
    return BlobMeasured(xoffb0, deg)

def printErr(blobErr: BlobMeasured):
    if blobErr is None:
        return
    img.draw_string(20, 10, f"Tilt: {blobErr.deg:.2f}", color=0, scale=1)
    if blobErr.deg < -5:
        img.draw_string(20, 20, "Turn Left", color=0, scale=1)
    elif blobErr.deg > 5:
        img.draw_string(20, 20, "Turn Right", color=0, scale=1)
    else:
        img.draw_string(20, 20, "Straight", color=0, scale=1)

    if blobErr.xoff < -10:
        img.draw_string(20, 30, "Shift Left", color=0, scale=1)
    elif blobErr.xoff > 10:
        img.draw_string(20, 30, "Shift Right", color=0, scale=1)
    else:
        img.draw_string(20, 30, "No Shift", color=0, scale=1)

def pid_ctrl(offset, angle, previous_error, previous_err_a, integral, dt):
    # Controller Coefficients
    kpo = 1.8
    kpa = 1.45
    kd = 0.75
    ki = 0.35

    icap = 0.5
    dt = dt if dt != 0 else 0.025

    # Normalize input errors
    e_off = offset / 40  # range -40 to 40
    e_ang = angle / 45

    integral += e_off * dt
    derivative = (e_off - previous_error) / dt + 0 * (e_ang - previous_err_a) / dt

    # Eliminate offset error when small (Deadband)
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

    # Reset integral if error crosses 0
    if (e_off * previous_error < 0):
        integral *= 0.5

    # Max/Min cap of integral
    if (integral > icap):
        integral = icap
    elif (integral < -icap):
        integral = -icap

    control = prop + ki * integral + kd * derivative
    return control, e_off, e_ang, integral

def ThrottleFromSteer(steering_angle):
    absAngle = abs(steering_angle)
    throttle = maxThrottle - (((maxThrottle - minThrottle) / 100) * absAngle)
    return throttle

# ==========================================
# MAIN LOOP
# ==========================================

past_off = 0
past_ang = 0
integral = 0
dt = 2
throttle_percent = minThrottle

while True:
    clock.tick()
    img = csi0.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else csi0.snapshot()
    line = img.get_regression([(255, 255) if BINARY_VISIBLE else THRESHOLD], robust=True)

    img.draw_rectangle(ROIs[0:4], color=127)
    img.draw_line(80, 0, 80, 160, color=127)

    if line:
        img.draw_line(line.line(), color=127)

        # Calculate Deflection Angle
        if line.y1() > line.y2():
            bottom_x, bottom_y = line.x1(), line.y1()
            top_x, top_y = line.x2(), line.y2()
        else:
            bottom_x, bottom_y = line.x2(), line.y2()
            top_x, top_y = line.x1(), line.y1()

        dx = top_x - bottom_x
        dy = bottom_y - top_y

        deflection_angle = math.degrees(math.atan2(dx, dy))

        # Makes the center of the screen as offset of 0
        offset = bottom_x - 40

        # Calculate PID
        control, past_off, past_ang, integral = pid_ctrl(offset, deflection_angle, past_off, past_ang, integral, dt)

        # Apply Deadband for perfectly straight lines
        if abs(control) < 0.08:
            car.Steer(car.STRAIGHT)
            throttle_percent = maxThrottle
        else:
            steer_percent = max(min(control, 1), -1) * 100
            if (steer_percent < 0):
                car.Steer(car.LEFT, abs(steer_percent))
            else:
                car.Steer(car.RIGHT, steer_percent)

            throttle_percent = ThrottleFromSteer(steer_percent)

        car.Throttle(car.FULL_SPEED_FORWARD, throttle_percent)

    else:
        # Failsafe / Blind Corner Push
        integral = 0
        car.Throttle(car.FULL_SPEED_FORWARD, blindThrottle)
        car.Steer(car.CURR_STEER, 100)
        continue

    dt = 1 / clock.fps()
