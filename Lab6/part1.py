# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# Lab 6, Part 2, Program 1: Multiple Blob Detection, Frame Rate Report, and LED Blinking

import csi
import image
import time
import math
from pyb import LED

red_LED = LED(1)

# threshold for white blob
thresholds = (180, 255)

# ROI: x y w h (weight)
ROIs = ([0, 0, 160, 10])

# set up sensor
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)

csi0.framesize(csi.QQVGA)
csi0.auto_gain(False)  # must be turned off for color tracking
csi0.auto_whitebal(False)  # must be turned off for color tracking

clock = time.clock()
count = 0


def findMiddleBlobs(img: csi.image) -> [None | image.Blob]:
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


def drawMiddleBlobs(img: csi.image, blobs: [None | image.Blob]) -> None:
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


while True:
    clock.tick()
    img = csi0.snapshot()

    # Find blob PARt 2 proram 1


    for blob in img.find_blobs(
        [thresholds], pixels_threshold=10, area_threshold=10, merge=True, roi=ROIs
    ):
        # These values depend on the blob not being circular - otherwise they will be shaky.
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
    img.draw_rectangle(ROIs, color=127)

    count += 1
    if count == 50:
        count = 0
        red_LED.toggle()

    print(clock.fps())

