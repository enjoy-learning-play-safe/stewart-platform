import numpy as np
from actuator_solving import actuator_solving
import write_read
import rotation_simple
import slicing_number_generator
from write_read import arduino
from config import *


def gcode(p_coor, x, y, z, roll, pitch, yaw, previous_inputs):
    start_pose = p_coor
    rott = np.matmul(rotation_simple(yaw, pitch, roll),p_origin_pbasis)
    p_coor = np.array([rott[0] + x, rott[1] + y, rott[2]+z]
                      ) - p_origin_pbasis + p_coor_home
    end_pose = p_coor

    slicing_number = slicing_number_generator(
        start_pose, end_pose)  # tune movement
    increment = slicing_number
    n = 0
    arduino.reset_input_buffer()
    print("Flush input buffer prior to movement")
    while slicing_number > 0:
        n = n + 1
        intermediate_x = (
            (x - previous_inputs[0])/increment)*n + previous_inputs[0]
        intermediate_y = (
            (y - previous_inputs[1])/increment)*n + previous_inputs[1]
        intermediate_z = (
            (z - previous_inputs[2])/increment)*n + previous_inputs[2]
        intermediate_roll = (
            (roll - previous_inputs[3]) / increment)*n + previous_inputs[3]
        intermediate_pitch = (
            (pitch - previous_inputs[4])/increment)*n + previous_inputs[4]
        intermediate_yaw = (
            (yaw - previous_inputs[5])/increment)*n + previous_inputs[5]
        rotated = np.matmul(rotation_simple(
            intermediate_yaw, intermediate_pitch, intermediate_roll),p_origin_pbasis)
        intermediate_p_coor = np.array([rotated[0] + intermediate_x, rotated[1] +
                                       intermediate_y, rotated[2]+intermediate_z]) - p_origin_pbasis + p_coor_home
        legs = actuator_solving(intermediate_p_coor)
        legs = np.round(legs, actuator_Precision)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(
            legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        print(output)
        slicing_number = slicing_number - 1
    print("End of slicing loop")
    print(f"Slices of movement is {increment}")
    print("Final GCode output should be:")
    print(output)
    write_read(output)
    write_read(output)
    write_read(output)

    return p_coor
