import serial
import time
import struct

arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

while True:
    # arduino.read_until('start')
    # data = arduino.read_until('end')
    data = arduino.readline()
    # data = arduino.read(4)
    print(data)
    # print(str(data).split(' '))
    if data != b'':
        # [a, b] = data.split(' ')
        # print(data.decode())
        [x] = struct.unpack('f', a)
        print(x) # printing the value