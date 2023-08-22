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

import cv2
import depthai as dai
from calc import HostSpatialsCalc
from utility import *
import numpy as np
import math
from Threading import Thread

angx, angy, angz, moisture = 0, 0, 0, 0;

imu = serial.Serial(port="/dev/ttyACM0",
                  baudrate=115200,
                  bytesize=8,
                  timeout=2,
                  writeTimeout = 2,
                  stopbits=serial.STOPBITS_ONE)


class Robot:
    def readIMU(self):
        global angx,angy,angz
        res = " "
        while res[0] != "IMU":
            res = imu.readline().deco0de('utf-8').split(" ")
            if len(res) != 4:
                continue
        angx = float(res[1])
        angy = float(res[2])
        angz = float(res[3])
        #print(f"x: {angx}")
        #print(f"y: {angy}")
        #print(f"z: {angz}")


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
                msg = f"Sense {angx} {angy} {angz}|"
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
        initial_z = angz
        # print(f"initial z: {initial_z}")
        diff = 0;
        self.turn_left(1)


        if initial_z > 0:
            while (diff <= 88):
                self.readIMU()
                z_temp = angz
                if(z_temp < 0):
                    z_temp += 360
                diff = initial_z - z_temp
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        else:
            while (diff <= 88):
                self.readIMU()
                diff = initial_z - angz;
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
        initial_z = angz
        # print(f"initial z: {initial_z}")
        diff = 0;
        self.turn_right(1)

        if initial_z < 0:
            while (diff <= 88):
                self.readIMU()
                z_temp = angz
                if(z_temp > 0):
                    z_temp = z_temp - 360
                diff = initial_z - z_temp
                if diff < 0:
                    diff *= -1
                #print(f"diff: {diff}")
        else:
            while (diff <= 88):
                self.readIMU()
                diff = initial_z - angz;
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
        

i2c = busio.I2C(board.SCL_1, board.SDA_1)
pca = PCA9685(i2c)
pca.frequency = 100

left_drive = DCMotor(pca.channels[7], pca.channels[6])#purple/gray
right_drive = DCMotor(pca.channels[9], pca.channels[8])#black/white
actuator = DCMotor(pca.channels[3], pca.channels[2]) #orange/yellow
wheel_motor = DCMotor(pca.channels[1], pca.channels[0]) #brown/red
drill = DCMotor(pca.channels[5], pca.channels[4])#green/blue
robot = Robot(left_drive, right_drive, actuator, wheel_motor, drill)


# Create pipeline


def create_grid(depthData, delta, disp, hostSpatials, x):
    x_temp = delta
    y_temp = delta
    grid_dic = {}
    distance_list = []
    coordinate_dic = {}
    row_num = int ((640) / (2 * delta))
    col_num = int ((700) / (2 * delta))
    for row in range(row_num):
        for col in range(col_num):
            text = TextHelper()
            hostSpatials.setDeltaRoi(delta)
            spatials, centroid = hostSpatials.calc_spatials(depthData, (x_temp,y_temp))

            if math.isnan(spatials['x']) or math.isnan(spatials['y']) or math.isnan(spatials['z']):
                pass

            else: 
                x_cord = float(("{:.1f}".format(spatials['x']/1000)))
                y_cord = math.cos(math.pi / 6) * float(("{:.1f}".format(spatials['y']/1000)))
                z_cord =  math.cos(math.pi / 6) * float(("{:.1f}".format(spatials['z']/1000)))
                distance = math.sqrt( (x_cord ** 2) + (y_cord ** 2) + (z_cord ** 2))
                grid_dic[distance] = [str(x_cord), str(y_cord),str(z_cord)]
                distance_list.append(distance)
                coordinate_dic[distance] = [x_temp, y_temp]
          
            x_temp = x_temp + (2 * delta)

        y_temp = y_temp + (2 * delta) 
        x_temp = x
    
    min_x_temp, min_y_temp= coordinate_dic.get(min(distance_list))
    text.rectangle(disp, (min_x_temp-delta, min_y_temp-delta), ( min_x_temp+delta, min_y_temp+delta))
    text.putText(disp, str(grid_dic.get(min(distance_list))), (min_x_temp + 20, min_y_temp + 20))
    return grid_dic.get(min(distance_list))

# Connect to device and start pipeline

def proximity_detection():
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    
    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    
    stereo.initialConfig.setConfidenceThreshold(255)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)

    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)

    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutDepth.setStreamName("disp")
    stereo.disparity.link(xoutDepth.input)

    with dai.Device(pipeline) as device:
        # Output queue will be used to get the depth frames from the outputs defined above
        depthQueue = device.getOutputQueue(name="depth")
        dispQ = device.getOutputQueue(name="disp")

        hostSpatials = HostSpatialsCalc(device)
  

        delta = 100
        y = delta
        x = delta

        print("Use WASD keys to move ROI.\nUse 'r' and 'f' to change ROI size.")

        data = []
        for index in range(15):   
            depthData = depthQueue.get()
            # Calculate spatial coordiantes from depth frame
            #spatials1, centroid1 = hostSpatials.calc_spatials(depthData, (x,y)) # centroid == x/y in our case
       
            # Get disparity frame for nicer depth visualization
            disp = dispQ.get().getFrame()
            disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

            min_dist = create_grid( depthData, delta, disp, hostSpatials, x)
            print(min_dist)
            #print(min_dist)
        
            if float(min_dist[2]) < 0.5: 
                #print(min_dist[2])
                if float(min_dist[0]) < 0:
                    data.append("Right")
                    #print(min_dist[0])
                    #print("Turn Right")
                else:
                    data.append("Left")
                    #print(min_dist[0])
                    #print("Turn Left")
            else:
                data.append("OK")
                #print("OK")iu7


        Left_count = data.count("Left")
        Right_count = data.count("Right")
        Ok_count = data.count("OK") 
	
        if(max(Left_count,Right_count,Ok_count) == Ok_count ):
            print("OK")
            return "OK"
        elif(max(Left_count,Right_count,Ok_count) == Left_count):
            print("Left")
            print(data)
            return "Left"
        else:
            print(data)
            print("Right")
            return "Right"
        data.clear() 
        min_dist.clear() 
        #return min_dist[2]
  
  
if(proximity_detection() == "Left"):
    robot.turn_left90()
    time.sleep(1)
    #blah = proximity_detection()
    if(proximity_detection() == "OK" ):
        robot.forward(1,delay=2)
        print("Right 1")
        time.sleep(1)
        robot.turn_right90()
        time.sleep(1)
        #blah = proximity_detection()
        if(proximity_detection() == "OK" ):
            robot.forward(1,delay=3)
            print("Right 2")
            time.sleep(1)
            robot.turn_right90()
            #blah = proximity_detection()
            if(proximity_detection() == "OK" ):
                robot.forward(1,delay=2)
                print("Left 1")
                time.sleep(1)
                robot.turn_left90()
                robot.forward(1,delay=2)
            else:
                print("stuck 1")
        else:
            print("stuck 2")
    else:
        print("stuck 8")
elif(proximity_detection() == "Right"):
    robot.turn_right90()
    time.sleep(1)
    blah = proximity_detection()
    if(proximity_detection() == "OK" ):
        robot.forward(1,delay=2.5)
        print("Right 6")
        time.sleep(1)
        robot.turn_left90()
        blah = proximity_detection()
        if(proximity_detection() == "OK" ):
            robot.forward(1,delay=2.5)
            print("Right 7")
            time.sleep(1)
            robot.turn_left90()
            blah = proximity_detection()
            if(proximity_detection() == "OK" ):
                robot.forward(1,delay=3)
                print("Left 4")
                time.sleep(1)
                robot.turn_right90()
                      
            else:
                print("stuck h_r")
        else:
            print("stuck b_r")
    
    elif(proximity_detection() == "Right"):
        robot.turn_right90()
        time.sleep(1)
        blah = proximity_detection()
        if(proximity_detection() == "OK" ):
            robot.forward(1,delay=2.5)
            print("Right 8")
            time.sleep(1)
            robot.turn_left90()
            blah = proximity_detection()
            if(proximity_detection() == "OK" ):
                robot.forward(1,delay=2)
                print("Right 9")
                time.sleep(1)
                robot.turn_left90()
                blah = proximity_detection()
                if(proximity_detection() == "OK" ):
                    robot.forward(1,delay=2.5)
                    print("Left 5")
                    time.sleep(1)
                    robot.turn_right90()
                    blah = proximity_detection()
                    if(proximity_detection() == "OK" ):
                        robot.forward(1,delay=2.5)
                        time.sleep(1)
                        print("Left 6")
                        robot.turn_left90()
                        blah = proximity_detection()
                        if(proximity_detection() == "OK" ):
                            robot.forward(1,delay=2.5)
                            time.sleep(1)
                            print("Right 10")
                            robot.turn_right90()
                        else:
                            print("stuck")
                    else:
                        print("stuck")
                else:
                    print("stuck h")
            else:
                print("stuck b")
        else:
            print("stuck c")
    else:
        print("stuck")  
else:
    robot.forward(1,delay=1)

    # Show the frame


    '''cv2.imshow("depth", disp)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break'''
'''def move_forward():
    nearest_obj = proximity_detection()
    robot.forward(1,  
'''

#robot.stop()
#robot.turn_right(1,2)
#robot.turn_left(1,4)
#robot.motion_demo()
#robot.forward(-1,3)

#robot.start_drilling()
#robot.fix_wheel(1,0.2)
#robot.fix_wheel(1,3.1)
#robot.lower_actuator(1, delay=20)
#robot.raise_actuator(1, delay=20)
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

'''elif(proximity_detection() == "Left"):
            robot.turn_left90()
            time.sleep(1)
            blah = proximity_detection()
            if(proximity_detection() == "OK" ):
                robot.forward(1,delay=2.5)
                print("Right 3")
                time.sleep(1)
                robot.turn_right90()
                time.sleep(1)
                blah = proximity_detection()
                if(proximity_detection() == "OK" ):
                    robot.forward(1,delay=2)
                    print("Right 4")
                    time.sleep(1)
                    robot.turn_right90()
                    blah = proximity_detection()
                    if(proximity_detection() == "OK" ):
                        robot.forward(1,delay=2.5)
                        print("Left 2")
                        time.sleep(1)
                        robot.turn_left90()
                        blah = proximity_detection()
                        if(proximity_detection() == "OK" ):
                            robot.forward(1,delay=2.5)
                            print("Right 5")
                            time.sleep(1)
                            robot.turn_right90()
                            blah = proximity_detection()
                            if(proximity_detection() == "OK" ):
                                robot.forward(1,delay=2.5)
                                print("Left 3")
                                time.sleep(1)
                                robot.turn_left90()
                            else:
                                print("stuck 3")
                        else:
                            print("stuck 4")
                    else:
                        print("stuck 5")
                else:
                    print("stuck 6")
            else:
                 print("stuck 7")'''


pca.deinit()

