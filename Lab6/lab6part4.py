# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Lab 6, Part 4: Deflection Angle Using Linear Regression

import sensor
import time
import math
from pyb import LED

red_LED = LED(1)
green_LED = LED(2)
blue_LED = LED(3)

# Changed to detect WHITE lines instead of dark lines
THRESHOLD = (180, 255)
BINARY_VISIBLE = True

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)  # 80x60 pixels
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
print_counter = 0

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    # Get robust linear regression
    line = img.get_regression([(255, 255) if BINARY_VISIBLE else THRESHOLD], robust=True)

    # Turn off all LEDs at the start of each frame
    red_LED.off()
    green_LED.off()
    blue_LED.off()

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

        position_string = "None"

        # Check if both endpoints are fully in the left region
        if x1 < 25 and x2 < 25:
            position_string = "Left"
            blue_LED.on()

        # Check if both endpoints are fully in the right region
        elif x1 > 55 and x2 > 55:
            position_string = "Right"
            red_LED.on()

        # Check if both endpoints are fully in the center region
        elif 25 <= x1 <= 55 and 25 <= x2 <= 55:
            position_string = "Center"
            green_LED.on()

        # If the endpoints fall in different regions, the line spans multiple regions
        else:
            position_string = "None"
            # LEDs remain off as set at the top of the loop

        # Serial Terminal Output (throttled to save FPS)
        print_counter += 1
        if print_counter >= 20:
            print(f"FPS: {clock.fps():.1f} | {deflection_angle:.2f} Degrees, \"{position_string}\"")
            print_counter = 0

    else:
        # No line detected at all
        print_counter += 1
        if print_counter >= 20:
            print(f"FPS: {clock.fps():.1f} | No Line Detected")
            print_counter = 0
