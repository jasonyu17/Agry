import board 
import busio
import ctypes
from adafruit_pca9685 import PCA9685
from adafruit_motor.motor import DCMotor
import serial
import time
import kivy
from kivy.app import App
from kivy.graphics import Color, Rectangle
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
import multiprocessing

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

    def spin_wheel180CW(self): #2 seconds for 90 degrees 
        self.wheel_motor.throttle = 1
        time.sleep(3.83)
        self.stop_wheel()

    def spin_wheel180CCW(self): #2 seconds for 90 degrees 
        self.wheel_motor.throttle = -1
        time.sleep(3.69)
        self.stop_wheel()
    def fix_wheel(self, speed, delay = 0):
        self.wheel_motor.throttle = speed
        if delay:
            time.sleep(delay)
            self.stop_wheel()
    def stop_wheel(self):
        self.wheel_motor.throttle = 0


    def start_drilling(self):
        self.lower_actuator(-1, delay=6)
        self.actuator.throttle =-.5
        self.drill.throttle = 1
        time.sleep(14)
        self.stop_actuator()
        self.stop_drill()
        self.raise_actuator(1, delay = 13)
        self.spin_wheel180CW()
        self.lower_actuator(-1, delay=10)
        time.sleep(1)
        self.raise_actuator(1, delay = 20)
        self.spin_wheel180CCW()
        

i2c = busio.I2C(board.SCL_1, board.SDA_1)
pca = PCA9685(i2c)
pca.frequency = 100

left_drive = DCMotor(pca.channels[7], pca.channels[6])#purple/gray
right_drive = DCMotor(pca.channels[9], pca.channels[8])#black/white
actuator = DCMotor(pca.channels[3], pca.channels[2]) #orange/yellow
wheel_motor = DCMotor(pca.channels[1], pca.channels[0]) #brown/red
drill = DCMotor(pca.channels[5], pca.channels[4])#green/blue
robot = Robot(left_drive, right_drive, actuator, wheel_motor, drill)


"""class ButtonApp(App):
    def callback1(self, event):
        robot.forward(1)
    def callback2(self, event):
        robot.stop()
    def callback3(self, event):
        robot.spin_wheel180CW(1, delay=4)
    def callback4(self, event):
        robot.spin_wheel180CCW(-1, delay=4)
    def build(self):
        Fl = FloatLayout()
        btn1 = Button(text ='Move Forward', size_h int =(.3, .5),
            background_color =(.3, .6, .7, 1),
            pos_hint ={'x':.2, 'y':.2 })
        btn1.bind(on_press=self.callback1)
        btn2 = Button(text ='Move Stop', size_hint =(.3, .5),
            background_color =(.3, .6, .7, 1),
            pos_hint ={'x':.5, 'y':.2 })
        btn2.bind(on_press=self.callback2)
        btn3 = Button(text ='Spin Wheel CW 180', size_hint =(.3, .5),
            background_color =(.3, .6, .7, 1),
            pos_hint ={'x':.2, 'y':.5 })
        btn3.bind(on_press=self.callback3)
        btn4 = Button(text ='Spin Wheel CW 180', size_hint =(.3, .5),
            background_color =(.3, .6, .7, 1),
            pos_hint ={'x':.5, 'y':.5 })
        btn4.bind(on_press=self.callback4)
        Fl.add_widget(btn1)
        Fl.add_widget(btn2)
        Fl.add_widget(btn3)
        Fl.add_widget(btn4)

        return Fl
 
    # callback function tells when button pressed
    
         
 
root = ButtonApp()
 

root.run()"""


robot.forward(1,10)

#robot.start_drilling()
#robot.fix_wheel(-1, .05)
#robot.lower_actuator(1, delay=5)
#robot.lower_actuator(1, delay=10)
"""robot.forward(1,delay=2)

robot.stop()

robot.lower_actuator(-1,delay=8)

robot.stop_actuator()

robot.raise_actuator(1,delay=8)

robot.stop_actuator()

robot.start_drill(1,delay=2)

robot.stop_drill()

robot.spin_wheel180CW(1, delay=4)

robot.spin_wheel180CCW(-1, delay=4)

robot.stop_wheel()"""



pca.deinit()

