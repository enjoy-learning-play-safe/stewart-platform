import time
from write_read import write_read

def echo():
    time.sleep(4)
    ping = 23
    while ping > 0:
        write_read("G90")
        ping = ping - 1