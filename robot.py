import board 
import busio
import ctypes
from adafruit_pca9685 import PCA9685
from adafruit_motor.motor import DCMotor
import time

class Robot:
    def __init__(self, left_drive, right_drive, actuator, wheel_motor, drill):
        self.left_drive = left_drive
        self.right_drive = right_drive
        self.actuator = actuator
        self.wheel_motor = wheel_motor
        self.drill = drill	

    def stop(self):
        self.left_drive.throttle = 0
        self.right_drive.throttle = 0

    def forward(self, speed, delay=None):
        self.left_drive.throttle = -1 * speed
        self.right_drive.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop()

    def backward(self, speed, delay=None):
        self.left_drive.throttle = speed
        self.right_drive.throttle = -1 * speed
        if delay:
            time.sleep(delay)
            self.stop()

    def turn_left(self, speed, delay=None):
        self.left_drive.throttle = speed
        self.right_drive.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop()

    def turn_right(self, speed, delay=None):
        speed = -1 * speed
        self.left_drive.throttle = speed
        self.right_drive.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop()

    def lower_actuator(self, speed, delay=None):
        self.actuator.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop_actuator()

    def raise_actuator(self, speed, delay=None):
        self.actuator.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop_actuator()

    def stop_actuator(self):
        self.actuator.throttle = 0

    def start_drill(self, speed, delay=None):
        self.drill.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop_drill()

    def stop_drill(self):
        self.drill.throttle = 0

i2c = busio.I2C(board.SCL_1, board.SDA_1)
pca = PCA9685(i2c)
pca.frequency = 100

left_drive = DCMotor(pca.channels[7], pca.channels[6])
right_drive = DCMotor(pca.channels[9], pca.channels[8])
actuator = DCMotor(pca.channels[5], pca.channels[4])
wheel_motor = DCMotor(pca.channels[3], pca.channels[2])
drill = DCMotor(pca.channels[1], pca.channels[0])

robot = Robot(left_drive, right_drive, actuator, wheel_motor, drill)

robot.forward(1,delay=2)

robot.stop()

#robot.lower_actuator(1,delay=8)

#robot.stop_actuator()

#robot.raise_actuator(-1,delay=8)

#robot.stop_actuator()

#robot.start_drill(1,delay=2)

#robot.stop_drill()

pca.deinit()

