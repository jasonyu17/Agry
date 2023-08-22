import serial
import time

x, y, z, moisture = 0, 0, 0, 0;

imu = serial.Serial(port="/dev/ttyACM0",
                    baudrate=115200,
                    bytesize=8,
                    timeout=2,
                    writeTimeout=2,
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


def readIMU():
    global x,y,z
    res = " "
    while res[0] != "IMU":
        res = imu.readline().decode('utf-8').split(" ")
        if len(res) != 4:
            continue
    x = float(res[1])
    y = float(res[2])
    z = float(res[3])
    print(f"x: {x}")
    print(f"y: {y}")
    print(f"z: {z}")


def readRaw():
    while 1:
        res = imu.readline().decode('utf-8')
        print(res)


def readGPS():
    res = " "
    while res[0] != "$GNGLL":
        res = gps.readline().decode('utf-8').split(",")
    print(res)
    print(f"Latitude: {res[1]}{res[2]}")
    print(f"Longitude: {res[3]}{res[4]}")


def readSensor():
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


def turn_left():
    # Enter code to turn left here
    print("start")
    readIMU();
    print("just read imu")
    initial_z = z
    print(f"initial z: {initial_z}")
    diff = 0;


    if initial_z > 0:
        while (diff <= 90):
            readIMU()
            z_temp = z
            if(z_temp < 0):
                z_temp += 360
            diff = initial_z - z_temp
            if diff < 0:
                diff *= -1
            print(f"diff: {diff}")
    else:
        while (diff <= 90):
            readIMU()
            diff = initial_z - z;
            if diff < 0:
                diff *= -1
            print(f"diff: {diff}")
    print("stop")


def turn_right():
    # Enter code to turn left here
    print("start")
    readIMU();
    print("just read imu")
    initial_z = z
    print(f"initial z: {initial_z}")
    diff = 0;

    if initial_z < 0:
        while (diff <= 90):
            readIMU()
            z_temp = z
            if(z_temp > 0):
                z_temp = z_temp - 360
            diff = initial_z - z_temp
            if diff < 0:
                diff *= -1
            print(f"diff: {diff}")
    else:
        while (diff <= 90):
            readIMU()
            diff = initial_z - z;
            if diff < 0:
                diff *= -1
            print(f"diff: {diff}")
    print("stop")

turn_right()
