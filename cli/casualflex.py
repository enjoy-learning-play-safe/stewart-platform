import actuator_solving
import write_read
import numpy as np
import time
import math
from menu import p_coor_home
from main import actuator_Precision

def casualflex():
    index = 180
    angle = 0
    cir_p_coor = p_coor_home
    n = 0
    while index > 0:
        n = n+1
        change = math.pi/90
        angle = angle + change
        x_coor = np.array([np.ones((6))])
        y_coor = np.array([np.ones((6))])
        z_coor = np.array([np.ones((6))])
        x_coor = x_coor * math.cos(angle)*60
        y_coor = y_coor * math.sin(angle)*60
        z_coor = z_coor * 0.5*n
        changes = np.concatenate((x_coor, y_coor, z_coor))
        cir_p_coor = p_coor_home + changes
        legs = actuator_solving(cir_p_coor)
        legs = np.round(legs, actuator_Precision)
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(
            legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        time.sleep(0.01)
        print(output)
        index = index-1
    print("done")
