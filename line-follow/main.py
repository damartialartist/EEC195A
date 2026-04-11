# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Lab 6, Program 2 & 3: Closest to Center ROIs and Angles of Deflection
import csi
import image
import time
import math

from pyb import Pin, Timer

# Classes --------------------


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
        # PWM
        self.tim = Timer(4, freq=100)
        self.pwmDCPos = self.tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=0)
        self.pwmServo = self.tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

    def Steer(self, mode, percentage=100):
        modifier = self.msToTicks((0.4 * (1-percentage/100)))

        if (mode == self.RIGHT):
            width = mode + modifier
        elif (mode == self.LEFT):
            width = mode - modifier
        else:
            width = mode

        self.pwmServo.pulse_width(int(width))

    def Throttle(self, mode, percentage=100):
        modifier = self.msToTicks((0.4 * (1-percentage/100)))

        if (mode == self.FULL_SPEED_REVERSE):
            width = mode + modifier
        elif (mode == self.FULL_SPEED_FORWARD):
            width = mode - modifier
        else:
            width = mode

        self.pwmDCPos.pulse_width(int(width))


# Camera Setup --------------------
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.QQQVGA)
csi0.auto_gain(False)
csi0.auto_whitebal(False)
img = csi0.snapshot()

# Car Initialization --------------------
myCar = Car()

# Constants --------------------
# IMG
img_width = img.width()
img_middle_x = img_width // 2

# Blobs
thresholds = (180, 255)
ROIs = (0, 0, 160, 10, 0.5)  # x y w h (weight)


THRESHOLD = (170, 255)
BINARY_VISIBLE = True

clock = time.clock()


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
    # These values are stable all the time.
    img.draw_rectangle(blob.rect(), color=127)
    img.draw_cross(blob.cx(), blob.cy(), color=127)
    # Note - the blob rotation is unique to 0-180 only.
    img.draw_keypoints(
        [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))],
        size=40,
        color=127,
    )

    return


def findBlobErr(blob):
    b0 = blob
    if b0 is None:
        return None
    deg = math.degrees(math.atan2(b0.cy() - 80, b0.cx() - 80)) - 90

    xoffb0 = b0.cx() - img_middle_x

    return BlobMeasured(xoffb0, deg)


deg = 0


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


def pid_ctrl(offset, angle, previous_error, integral, dt):
    # define ctrller coeffs
    kpo = 1
    kpa = 0.5
    kd = 0
    ki = 0

    # normalize input errors; desired offset and angle are both 0
    e_off = -1 * offset / 40  # range -40 to 40
    e_ang = -1 * angle / 90

    prop = kpo * e_off + kpa * e_ang
    integral += e_off * dt
    derivative = (e_off - previous_error) / dt

    # control = prop + ki * integral + kd * derivative
    control = prop
    return control, e_off, integral


past_err = 0
integral = 0
dt = 1 / clock.fps()
while True:
    clock.tick()
    img = csi0.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else csi0.snapshot()

    line = img.get_regression([(255, 255) if BINARY_VISIBLE else THRESHOLD], robust=True)

    img.draw_rectangle(ROIs[0:4], color=127)

    img.draw_line(80, 0, 80, 160, color=127)

    # blobs = findTopBlob(img)

    # if blobs:
    #     drawTopBlob(img, blobs)
    #     blobErr = findBlobErr(blobs)

    # printErr(blobErr)

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

        # Calculate angle relative to the Y-axis (Straight up = 0 degrees)
        deflection_angle = math.degrees(math.atan2(dx, dy))

        # Rule of Thirds Logic (QQQVGA width = 80)
        x1, x2 = line.x1(), line.x2()

        # position_string = "None"

        # # Check if both endpoints are fully in the left region
        # if x1 < 25 and x2 < 25:
        #     position_string = "Left"

        # # Check if both endpoints are fully in the right region
        # elif x1 > 55 and x2 > 55:
        #     position_string = "Right"

        # # Check if both endpoints are fully in the center region
        # elif 25 <= x1 <= 55 and 25 <= x2 <= 55:
        #     position_string = "Center"

        # # If the endpoints fall in different regions, the line spans multiple regions
        # else:
        #     position_string = "None"
        #     # LEDs remain off as set at the top of the loop

        # print(f"FPS: {clock.fps():.1f} | {deflection_angle:.2f} Degrees, \"{position_string}\"")
        # print(f"Bottom:{x1}, Top:{x2}")
        # Serial Terminal Output (throttled to save FPS)

        # Makes the center of the screen as offset of 0
        offset = x1 - 40

        # control, past_err, integral = pid_ctrl(offset, deflection_angle, past_err, integral, dt)
        # steer_percent = max(min(control, 1), -1) * 100
        # print(f"Control RetVal: {steer_percent}%")
        # Car.Steer(Car, what to put here, steer_percent)

    else:
        print(f"FPS: {clock.fps():.1f} | No Line Detected")

    print(f"FPS: {clock.fps():.1f}")

    dt = 1 / clock.fps()
