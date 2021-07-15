import math
import rotation_simple
import actuator_solving
import time
from config import *
import write_read
import numpy as np

def rotatingflex():
    index = 180
    angle = 0
    n = 0
    while index > 0:
        n = n+1
        change = math.pi/90
        angle = angle + change
        x_coor = math.cos(angle)*30
        y_coor = math.sin(angle)*30
        pitch =  math.cos(angle)* (-30/180)*math.pi
        roll = math.sin(angle)* (30/180)*math.pi
        rott = np.matmul(rotation_simple(0, pitch, roll),p_origin_pbasis)
        p_coor = np.array([rott[0] + x_coor, rott[1] +y_coor, rott[2]]) - p_origin_pbasis + p_coor_home
        legs = actuator_solving(p_coor)
        legs = np.round(legs, actuator_Precision) 
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        time.sleep(0.01)
        # print(np.round(p_coor,2))
        print(output)
        index = index -1
    print("done") 