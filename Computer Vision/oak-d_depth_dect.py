import cv2
import depthai as dai
from calc import HostSpatialsCalc
from utility import *
import numpy as np
import math


# Create pipeline
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

def create_grid(delta, disp):
    x_temp = delta
    y_temp = delta
    grid_dic = {}
    distance_list = []
    coordinate_dic = {}
    row_num = int (640 / ( 2 * delta))
    col_num = int (700 / ( 2 * delta))
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
with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth")
    dispQ = device.getOutputQueue(name="disp")

    hostSpatials = HostSpatialsCalc(device)
  

    delta = int(input("Enter Delta: "))
    y = delta
    x = delta
    step = 5

    print("Use WASD keys to move ROI.\nUse 'r' and 'f' to change ROI size.")

    while True:
        depthData = depthQueue.get()
        # Calculate spatial coordiantes from depth frame
        #spatials1, centroid1 = hostSpatials.calc_spatials(depthData, (x,y)) # centroid == x/y in our case
       
        # Get disparity frame for nicer depth visualization
        disp = dispQ.get().getFrame()
        disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

        min_dist = create_grid( delta, disp)
        #print(min_dist)
        if float(min_dist[2]) < 0.5: 
            if float(min_dist[0]) < 0:
                print("Turn Right")
            elif float(min_dist[0]) > 0:
                print("Turn Left")
            else:
                print("Turn Left or Turn Right")
        else:
            print("OK")
      
        # Show the frame

        cv2.imshow("depth", disp)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        '''elif key == ord('w'):
            y -= step
        elif key == ord('a'):
            x -= step
        elif key == ord('s'):
            y += step
        elif key == ord('d'):
            x += step
        elif key == ord('r'): # Increase Delta
            if delta < 50:
                delta += 1
                hostSpatials.setDeltaRoi(delta)
        elif key == ord('f'): # Decrease Delta
            if 3 < delta:
                delta -= 1
                hostSpatials.setDeltaRoi(delta)'''
