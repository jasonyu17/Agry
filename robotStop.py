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


x, y, z, moisture = 0, 0, 0, 0;

imu = serial.Serial(port="/dev/ttyACM0",
                  baudrate=115200,
                  bytesize=8,
                  timeout=2,
                  writeTimeout = 2,
                  stopbits=serial.STOPBITS_ONE)

# sensor = serial.Serial(port="COM7",
#                        baudrate=115200,
#                        bytesize=8,
#                        timeout=2,
#                        writeTimeout = 2,
#                        stopbits=serial.STOPBITS_ONE)

# gps = serial.Serial(port="COM3",
#                   baudrate=9600,
#                   bytesize=8,
#                   timeout=2,
#                   stopbits=serial.STOPBITS_ONE)


class Robot:
    def readIMU(self):
        global x,y,z
        res = " "
        while res[0] != "IMU":
            res = imu.readline().decode('utf-8').split(" ")
        x = float(res[1])
        y = float(res[2])
        z = float(res[3])
        #print(f"x: {x}")
        #print(f"y: {y}")
        #print(f"z: {z}")


    def readRaw(self):
        while 1:
            res = imu.readline().decode('utf-8')
            print(res)


    def readGPS(self):
        res = " "
        while res[0] != "$GNGLL":
            res = gps.readline().decode('utf-8').split(",")
        print(res)
        print(f"Latitude: {res[1]}{res[2]}")
        print(f"Longitude: {res[3]}{res[4]}")


    def readSensor(self):
        gotMoisture = False
        while gotMoisture == False:
            try:
                msg = f"Sense {x} {y} {z}|"
                sensor.write(msg.encode())
                print("writing sense")
            except Exception as e:
                print(e)

            res = sensor.readline().decode().split(" ")
            print(res)
            if (res[0] == "Moisture"):
                moisture = float(res[1])
                gotMoisture = True
                print("success")
        gotMoisture = False


    def turn_left90(self):
        # Enter code to turn left here
        # print("start")
        self.readIMU();
        # print("just read imu")
        initial_z = z
        # print(f"initial z: {initial_z}")
        diff = 0;
        self.turn_left(1)


        if initial_z > 0:
            while (diff <= 88):
                self.readIMU()
                z_temp = z
                if(z_temp < 0):
                    z_temp += 360
                diff = initial_z - z_temp
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        else:
            while (diff <= 88):
                self.readIMU()
                diff = initial_z - z;
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        # print("stop")
        print(f"diff: {diff}")
        self.stop()

    def turn_right90(self):
        # Enter code to turn left here
        # print("start")
        self.readIMU();
        # print("just read imu")
        initial_z = z
        # print(f"initial z: {initial_z}")
        diff = 0;
        self.turn_right(1)

        if initial_z < 0:
            while (diff <= 88):
                self.readIMU()
                z_temp = z
                if(z_temp > 0):
                    z_temp = z_temp - 360
                diff = initial_z - z_temp
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        else:
            while (diff <= 88):
                self.readIMU()
                diff = initial_z - z;
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        #print("stop")
        print(f"diff: {diff}")
        self.stop()


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
        self.left_drive.throttle = -1* speed
        self.right_drive.throttle = -1*speed
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

    def motion_demo(self):
        self.forward(1,10)
        self.turn_left(1,10)
        self.forward(1,10)
'''
    def turn_left90(self):
        self.turn_left(1)
        print("start")
        junk = imu.readline().decode('utf-8').split(" ")
        res = imu.readline().decode('utf-8').split(" ")
        # Check if the list has enough elements before accessing them
        if len(res) < 2:
            print("Incomplete data received")
            return
        initial_yaw = float(res[2])
        print(initial_yaw)
    
        while (res[0] != "IMU"):
            junk = imu.readline().decode('utf-8').split(" ")
            res = imu.readline().decode('utf-8').split(" ")
            print(res)
            res[0] = float(res[0])
            res[1] = float(res[1])
            res[2] = float(res[2])
            print(res[2])
            if res[2] > initial_yaw + 87:
                self.stop()
                print("stop turn")
                break
        
    def turn_right90(self):
        self.turn_right(1)
        print("start")
        junk = imu.readline().decode('utf-8').split(" ")
        res = imu.readline().decode('utf-8').split(" ")
        # Check if the list has enough elements before accessing them
        if len(res) < 2:
            print("Incomplete data received")
            return
        initial_yaw = float(res[2])
    
        while (res[0] != "IMU"):
            junk = imu.readline().decode('utf-8').split(" ")
            res = imu.readline().decode('utf-8').split(" ")
            res[0] = float(res[0])
            res[1] = float(res[1])
            res[2] = float(res[2])
            if res[2] < initial_yaw - 87:
                self.stop()
                print("stop turn")
                break
 '''       

i2c = busio.I2C(board.SCL_1, board.SDA_1)
pca = PCA9685(i2c)
pca.frequency = 100

left_drive = DCMotor(pca.channels[7], pca.channels[6])#purple/gray
right_drive = DCMotor(pca.channels[9], pca.channels[8])#black/white
actuator = DCMotor(pca.channels[3], pca.channels[2]) #orange/yellow
wheel_motor = DCMotor(pca.channels[1], pca.channels[0]) #brown/red
drill = DCMotor(pca.channels[5], pca.channels[4])#green/blue
robot = Robot(left_drive, right_drive, actuator, wheel_motor, drill)


robot.stop()
robot.stop_actuator()
robot.stop_drill()
robot.stop_wheel()

#robot.turn_right(1,3)

#robot.motion_demo()
#robot.forward(1,10)#

'''while True:
    res = imu.readline().decode('utf-8').split(" ")
    print(res)'''

#robot.start_drilling()
#robot.fix_wheel(-1, 2)
#robot.turn_left90()
#time.sleep(2)
#robot.turn_right90()
#robot.lower_actuator(1, delay=4)
#robot.lower_actuator(-1, delay=10)
#robot.raise_actuator(1, delay=10)
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

# robot.turn_left90()
time.sleep(1)

pca.deinit()

