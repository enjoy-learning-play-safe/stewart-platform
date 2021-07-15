import time
from parameters import arduino


def write_read(x):
    x = str(x) + "\r\n"
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.03)
    data = arduino.readline()
    print(data)
    return