import serial
import time

arduino = serial.Serial(port='COM4', baudrate=250000, timeout=0.02)

def write_read(x):
    x = str(x) + "\r"
    arduino.write(bytes(x, 'utf-8'))
    #arduino.writelines(bytes(x, 'utf-8'))
    time.sleep(0.02)
    data = arduino.readline()
    print(data)
    return