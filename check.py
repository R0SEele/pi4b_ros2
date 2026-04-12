import serial
import binascii

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

while True:
    data = ser.read(64)
    if data:
        print(binascii.hexlify(data).decode())Q