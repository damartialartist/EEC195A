# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Lab 6, Part 3: Line Detecion and Pulse Width Modulation

import sensor
import image
import time
import math
from pyb import Pin, Timer, LED

red_LED = LED(1)
green_LED = LED(2)
blue_LED = LED(3)

# Setup Timer
tim = Timer(4, freq=100)

# Channel 1 is locked to P7 (DC Motor)
pwmDC = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=0)

# Channel 2 is locked to P8 (Servo)
pwmServo = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

# Helper function to convert microseconds to timer ticks
# Based on a 100Hz frequency (10,000 us period)
def usToTicks(us: float) -> int:
    return int((us / 10000.0) * 19200)


# threshold for white blob
thresholds = (180, 255)


# ROI: x y w h (weight)
ROIs = [(0, 0, 160, 10, 0.5), (0, 80, 160, 10, 0.5)]

# set up sensor
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)  # Give the camera time to adjust
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking

clock = time.clock()
print_counter = 0


def findMiddleBlobs(img) -> [None | image.Blob]:
    img_middle_x = img.width() / 2
    middle_blobs = []
    for r in ROIs:
        blob_array = img.find_blobs(
            [thresholds], pixels_threshold=5, area_threshold=5, merge=True, roi=r[0:4]
        )
        if len(blob_array) == 0:
            middle_blobs.append(None)
            continue

        best_blob = None
        min_diff = float('inf')

        for b in blob_array:
            diff = abs(b.cx() - img_middle_x)
            if diff < min_diff:
                best_blob = b
                min_diff = diff
        middle_blobs.append(best_blob)

    return middle_blobs


def drawMiddleBlobs(img, blobs: [None | image.Blob]) -> None:
    cxs = []
    cys = []
    for blob in blobs:
        if blob is None:
            continue
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
        cxs.append(blob.cx())
        cys.append(blob.cy())

    if (len(cxs) == 2 and len(cys) == 2):
        img.draw_line(cxs[0], cys[0], cxs[1], cys[1], color=0)

    return


def blob_angle(blobs: [None | image.Blob]) -> float | None:
    cxs = []
    cys = []
    for blob in blobs:
        if blob is None:
            return None
        cxs.append(blob.cx())
        cys.append(blob.cy())
    rad = math.atan2(cys[1] - cys[0], cxs[1] - cxs[0])
    return math.degrees(rad) - 90


while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = findMiddleBlobs(img)
    drawMiddleBlobs(img, blobs)
    deg = blob_angle(blobs)

    # Draw ROIs
    img.draw_rectangle(ROIs[0][0:4], color=127)
    img.draw_rectangle(ROIs[1][0:4], color=127)
    img.draw_line(80, 0, 80, 160, color=127)

    if deg:
        img.draw_string(20, 10, f"Tilt: {deg:.2f}", color=10, scale=1)

    # Turn all LEDs off at the start of the frame
    red_LED.off()
    green_LED.off()
    blue_LED.off()

    print_counter += 1

    # Grab the blob closest to the car (the bottom ROI)
    bottom_blob = blobs[1] if len(blobs) > 1 else None

    if bottom_blob is None:
        # Track not detected
        pwmDC.pulse_width(usToTicks(1500))
        # Servo maintains last pulse width (do nothing to pwmServo)
        if print_counter >= 20:
            print(f"FPS: {clock.fps():.1f} | NO TRACK | Servo: KEEP | DC: 1500us | LEDs: OFF")

    else:
        cx = bottom_blob.cx()

        if cx < 53:
            # Track on the left
            pwmServo.pulse_width(usToTicks(1100))
            pwmDC.pulse_width(usToTicks(1575))
            blue_LED.on()
            if print_counter >= 20:
                print(f"FPS: {clock.fps():.1f} | LEFT | Servo: 1100us | DC: 1575us | LED: BLUE")

        elif cx > 106:
            # Track on the right
            pwmServo.pulse_width(usToTicks(1900))
            pwmDC.pulse_width(usToTicks(1575))
            red_LED.on()
            if print_counter >= 20:
                print(f"FPS: {clock.fps():.1f} | RIGHT | Servo: 1900us | DC: 1575us | LED: RED")

        else:
            # Track in the Center
            pwmServo.pulse_width(usToTicks(1500))
            pwmDC.pulse_width(usToTicks(1650))
            green_LED.on()
            if print_counter >= 20:
                print(f"FPS: {clock.fps():.1f} | CENTER | Servo: 1500us | DC: 1650us | LED: GREEN")

    # Reset the print counter
    if print_counter >= 20:
        print_counter = 0
