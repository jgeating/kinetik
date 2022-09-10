import serial
import time
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    
    data = arduino.readline()
    return data
while True:
    # arduino.read_until('start')
    # data = arduino.read_until('end')
    data = arduino.readline()

    # num = input("Enter a number: ") # Taking input from user
    # value = write_read(num)
    print(str(data).split(' '))
    print(str(data)) # printing the value