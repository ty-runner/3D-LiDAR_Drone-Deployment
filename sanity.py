import serial
import time

ser = serial.Serial("/dev/ttyTHS1", baudrate=115200, timeout=0.1)

print("opened")

ser.write(b'\x55\x55')
time.sleep(0.1)

ser.close()
