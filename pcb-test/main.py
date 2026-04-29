# Jacky Chen, Mandy Chokry, Angel Hernandez Vega, Nathan Leung
# PCB Testing

import csi
import pyb
from pyb import Pin, Timer
import time


maxThrottle = 48
minThrottle = 22
blindThrottle = 20


# Classes --------------------

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
        # PWM
        self.tim = Timer(4, freq=500)
        self.pwmDCPos = self.tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width=0)
        self.pwmServo = self.tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width=0)

        # INA INB
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
        if (mode == self.BRAKE):
            self.ina.value(0)
            self.inb.value(0)
            self.pwmDCPos.pulse_width_percent(0)
        elif (mode == self.FULL_SPEED_FORWARD):
            self.ina.value(1)
            self.inb.value(0)
            self.pwmDCPos.pulse_width_percent(30)


# Camera Setup --------------------
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.QQQVGA)
csi0.auto_gain(False, gain_db=18.5)
csi0.auto_whitebal(False)
img = csi0.snapshot()

# Car Initialization --------------------
car = Car()
car.Throttle(car.BRAKE)
#car.Steer(car.STRAIGHT)

clock = time.clock()

while True:
    clock.tick()
    img = csi0.snapshot()

    print("yamom")
    pyb.delay(5000)
    print("yamom")
    car.Throttle(car.FULL_SPEED_FORWARD)
    pyb.delay(5000)
    car.Throttle(car.BRAKE)
    print(clock.fps())
