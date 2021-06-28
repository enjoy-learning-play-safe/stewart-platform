
def home(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, previous_inputs):
    slicing_number = 15  # tune movement
    increment = slicing_number
    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    n = 0
    # print("Initiating homing process")
    # write_read("G28")                                 #G28 Homing process, implement after configuring Marlin Quick_Home
    # time.sleep(1)
    print("Moving all legs to midpoint")
    while slicing_number > 0:
        n = n+1
        inc_x = ((x - previous_inputs[0])/increment)*n + previous_inputs[0]
        inc_y = ((y - previous_inputs[1])/increment)*n + previous_inputs[1]
        inc_z = ((z - previous_inputs[2])/increment)*n + previous_inputs[2]
        inc_roll = ((roll - previous_inputs[3]) /
                    increment)*n + previous_inputs[3]
        inc_pitch = (
            (pitch-previous_inputs[4])/increment)*n + previous_inputs[4]
        inc_yaw = ((yaw-previous_inputs[5])/increment)*n + previous_inputs[5]
        rotated = np.matmul(rotation_simple(
            inc_yaw, inc_pitch, inc_roll), p_coor_pbasis)
        final_p_coor = np.array(
            [rotated[0] + inc_x, rotated[1] + inc_y, rotated[2] + inc_z]) - p_origin_pbasis + p_coor
        legs = actuator_solving(b_coor, final_p_coor)
        legs = np.round(legs, 4)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(legs[2]) + \
            " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        print(output)
        slicing_number=slicing_number-1 
    write_read(output)  
    previous_inputs= np.zeros((6))
    return