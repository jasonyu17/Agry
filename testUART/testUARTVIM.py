import serial
import time

arduino = serial.Serial(
	port = '/dev/ttyACM0',
	baudrate = 115200,
	bytesize = serial.EIGHTBITS,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	timeout = 5,
	xonxoff = False,
	rtscts = False,
	dsrdtr = False,
	writeTimeout = 2
)

while True:
	try: 
		arduino.write("Cmd from Jetson|".encode())
		data = arduino.readline()
		if data:
			print(data)
		time.sleep(1)
	except Exception as e:
		print(e)
		arduino.close()

