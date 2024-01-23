import serial


port = "/dev/ttyACM1"

ser = serial.Serial(port, 115200, timeout=10)


#print serial data
while True:
    print(ser.readline())
