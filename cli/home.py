import numpy as np
from actuator_solving import actuator_solving
from write_read import write_read
from rotation_simple import rotation_simple
from menu import p_coor_home
from main import actuator_Precision
import slicing_number_generator



def home(p_coor, previous_inputs):
    # arduino.reset_input_buffer()
    x = y = z = roll = pitch = yaw = 0
    start_pose = p_coor
    end_pose = p_coor_home

    slicing_number = slicing_number_generator(
        start_pose, end_pose)  # tune movement
    increment = slicing_number

    n = 0
    print("Moving all legs to midpoint")
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
        rotated = np.matmul(rotation_simple(intermediate_yaw, intermediate_pitch, intermediate_roll),p_origin_pbasis)
            
        intermediate_p_coor = np.array([rotated[0] + intermediate_x, rotated[1] +
                                       intermediate_y, rotated[2] + intermediate_z]) - p_origin_pbasis + p_coor_home
        legs = actuator_solving(intermediate_p_coor)
        legs = np.round(legs, actuator_Precision)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(
            legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        print(output)
        slicing_number = slicing_number-1
    print(f"Slices of movement is {increment}")
    write_read(output)
    previous_inputs = np.zeros((6))
    return p_coor_home
