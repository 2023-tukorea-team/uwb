import serial

ser = serial.Serial('/dev/ttyS0', 9600)

while True:
	try:
		distance = ser.readline().decode('utf-8').strip();
		if len(distance) == 0 :
			continue;
		print("Received:", distance);
	except KeyboardInterrupt:
		ser.close();
		print("Serial communication closed.");
		break;
	except Exception:
		ser.close();
		ser = serial.Serial('/dev/ttyS0', 9600)
		pass;
