import numpy as np


def gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x, y, z, roll, pitch, yaw, previous_inputs):
    slicing_number = 25  # tune movement
    increment = slicing_number
    n = 0
    while slicing_number > 0:
        n = n + 1
        inc_x = ((x - previous_inputs[0])/increment)*n + previous_inputs[0]
        inc_y = ((y - previous_inputs[1])/increment)*n + previous_inputs[1]
        inc_z = ((z - previous_inputs[2])/increment)*n + previous_inputs[2]
        inc_roll = ((roll - previous_inputs[3]) /
                    increment)*n + previous_inputs[3]
        inc_pitch = (
            (pitch - previous_inputs[4])/increment)*n + previous_inputs[4]
        inc_yaw = ((yaw - previous_inputs[5])/increment)*n + previous_inputs[5]
        rotated = np.matmul(rotation_simple(
            inc_yaw, inc_pitch, inc_roll), p_coor_pbasis)
        final_p_coor = np.array(
            [rotated[0] + inc_x, rotated[1] + inc_y, rotated[2]+inc_z]) - p_origin_pbasis + p_coor
        legs = actuator_solving(b_coor, final_p_coor)
        legs = np.round(legs, 3)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(legs[2]) + \
            " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        print(output)
        slicing_number = slicing_number - 1
    print("End of slicing loop")
    print("Final GCode output should be:")
    print(output)
    write_read(output)
    # write_read(output)
    # write_read(output)
    return