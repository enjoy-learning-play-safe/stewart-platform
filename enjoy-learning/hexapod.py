import math
import numpy as np
import serial
import time

arduino = serial.Serial(port='COM3', baudrate=250000, timeout=0.2)


def write_read(x):
    x = str(x) + "\r\n"
    arduino.write(bytes(x, 'utf-8'))
    # arduino.writelines(bytes(x, 'utf-8'))
    time.sleep(0.03)
    data = arduino.readline()
    print(data)
    return


def start():
    time.sleep(2)
    write_read("start")


def echo():
    time.sleep(4)
    ping = 23
    while ping > 0:
        write_read("G90")
        ping = ping - 1


def casualflex(b_coor, p_coor):
    # move in a circle radius 60mm
    # index every 1 degrees, cycle of 360
    # reference to a point of z+100
    # bring in p_coor, radius of platform
    index = 180
    angle = 0
    cir_p_coor = p_coor
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
        cir_p_coor = p_coor + changes
        legs = actuator_solving(b_coor, cir_p_coor)
        legs = np.round(legs, actuator_Precision)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(
            legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        time.sleep(0.01)
        print(output)
        index = index-1
    print("done")


def recasualflex(b_coor, p_coor):
    # move in a circle radius 60mm
    # index every 1 degrees, cycle of 360
    # reference to a point of z+100
    # bring in p_coor, radius of platform
    index = 180
    angle = 0
    cir_p_coor = p_coor
    n = 0
    while index > 0:
        n = n+1
        change = math.pi/90
        angle = angle - change
        x_coor = np.array([np.ones((6))])
        y_coor = np.array([np.ones((6))])
        z_coor = np.array([np.ones((6))])*90
        x_coor = x_coor * math.cos(angle)*60
        y_coor = y_coor * math.sin(angle)*60
        z_coor = z_coor - 0.5*n
        changes = np.concatenate((x_coor, y_coor, z_coor))
        cir_p_coor = p_coor + changes
        legs = actuator_solving(b_coor, cir_p_coor)
        legs = np.round(legs, actuator_Precision)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(
            legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        time.sleep(0.01)
        print(output)
        index = index-1
    print("done")


def rotation_simple(psi, theta, phi):
    cpsi = math.cos(psi)
    ctheta = math.cos(theta)
    cphi = math.cos(phi)
    spsi = math.sin(psi)
    stheta = math.sin(theta)
    sphi = math.sin(phi)
    rota = np.array([[cpsi*ctheta, (cpsi*stheta*sphi - spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)], [spsi*ctheta,
                    (cpsi*cphi + spsi*stheta*sphi), (spsi*stheta*cphi - cpsi*sphi)], [-stheta, ctheta*sphi, ctheta*cphi]])
    # [cpsi*ctheta, (cpsi*stheta*sphi-spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)]
    # [spsi*ctheta, (cpsi*cphi+spsi*stheta*sphi), (spsi*stheta*cphi-cpsi*sphi)]
    # [-stheta, ctheta*sphi, ctheta*cphi]
    return rota


def actuator_solving(b_coor, p_coor):
    leg1 = p_coor[2][0] - (abs(fixed_rods**2 - (p_coor[0][0] - b_coor[0][0]) **
                               2 - (p_coor[1][0] - b_coor[1][0])**2))**0.5  # not ready for non hexapod
    leg2 = p_coor[2][1] - (abs(fixed_rods**2 - (p_coor[0][1] -
                                                b_coor[0][1])**2 - (p_coor[1][1] - b_coor[1][1])**2))**0.5
    leg3 = p_coor[2][2] - (abs(fixed_rods**2 - (p_coor[0][2] -
                                                b_coor[0][2])**2 - (p_coor[1][2] - b_coor[1][2])**2))**0.5
    leg4 = p_coor[2][3] - (abs(fixed_rods**2 - (p_coor[0][3] -
                                                b_coor[0][3])**2 - (p_coor[1][3] - b_coor[1][3])**2))**0.5
    leg5 = p_coor[2][4] - (abs(fixed_rods**2 - (p_coor[0][4] -
                                                b_coor[0][4])**2 - (p_coor[1][4] - b_coor[1][4])**2))**0.5
    leg6 = p_coor[2][5] - (abs(fixed_rods**2 - (p_coor[0][5] -
                                                b_coor[0][5])**2 - (p_coor[1][5] - b_coor[1][5])**2))**0.5
    leggy = np.array([leg1, leg2, leg3, leg4, leg5, leg6])
    return leggy


def slicing_number_generator(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x, y, z, roll, pitch, yaw):

    # p_coor is already the previous platform coordinate before movement. no need to solve it again
    previous_legs = actuator_solving(b_coor, p_coor)
    previous_legs = np.round(previous_legs, actuator_Precision)
    rotated_Final = np.matmul(rotation_simple(yaw, pitch, roll), p_coor_pbasis)
    final_p_coor = np.array([rotated_Final[0] + x, rotated_Final[1] + y,
                            rotated_Final[2] + z]) - p_origin_pbasis + p_coor
    final_legs = actuator_solving(b_coor, final_p_coor)
    final_legs = np.round(final_legs, actuator_Precision)
    actuator_change = []
    zip_object = zip(final_legs, previous_legs)
    for final_legs_i, previous_legs_i in zip_object:
        actuator_change.append(abs(final_legs_i - previous_legs_i))
    max_actuator_change = max(actuator_change)
    slicing_number = int(
        math.ceil(max_actuator_change / float(max_change_per_slice)))
    if(slicing_number >= minimum_slice_per_movement):
        return slicing_number
    else:
        return minimum_slice_per_movement


def home(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, previous_inputs):
    # arduino.reset_input_buffer()
    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    slicing_number = slicing_number_generator(
        p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x, y, z, roll, pitch, yaw)  # tune movement
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
        intermediate_roll = ((roll - previous_inputs[3]) /
                             increment)*n + previous_inputs[3]
        intermediate_pitch = (
            (pitch - previous_inputs[4])/increment)*n + previous_inputs[4]
        intermediate_yaw = (
            (yaw - previous_inputs[5])/increment)*n + previous_inputs[5]
        rotated = np.matmul(rotation_simple(
            intermediate_yaw, intermediate_pitch, intermediate_roll), p_coor_pbasis)
        intermediate_p_coor = np.array(
            [rotated[0] + intermediate_x, rotated[1] + intermediate_y, rotated[2] + intermediate_z]) - p_origin_pbasis + p_coor
        legs = actuator_solving(b_coor, intermediate_p_coor)
        legs = np.round(legs, actuator_Precision)  # increase precison here
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(legs[2]) + \
            " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        print(output)
        slicing_number = slicing_number-1
    print(f"Slices of movement is {increment}")
    write_read(output)
    previous_inputs = np.zeros((6))
    return


def gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x, y, z, roll, pitch, yaw, previous_inputs):
    slicing_number = slicing_number_generator(
        p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x, y, z, roll, pitch, yaw)  # tune movement
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
            intermediate_yaw, intermediate_pitch, intermediate_roll), p_coor_pbasis)
        intermediate_p_coor = np.array([rotated[0] + intermediate_x, rotated[1] +
                                       intermediate_y, rotated[2]+intermediate_z]) - p_origin_pbasis + p_coor
        legs = actuator_solving(b_coor, intermediate_p_coor)
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
    return


def menu():
    state = 0
    while state == 0:

        x_translate = 0
        y_translate = 0
        z_translate = 0
        roll = 0
        pitch = 0
        yaw = 0
        previous_inputs = np.array(
            [x_translate, y_translate, z_translate, roll, pitch, yaw])
        num_legs = 6      # num_legs= int(input("Number of legs: "))
        if num_legs == 6:
            p_angles = np.array(
                [0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3])
            p_coorxy = np.array([np.cos(p_angles)*p_r, np.sin(p_angles)*p_r])
            p_coor_pbasis = np.append(
                p_coorxy, np.array([np.zeros(6)]), axis=0)
            b_leg2x = p_coorxy[0][1]
            b_leg3x = p_coorxy[0][2]
            b_leg23y = (b_r**2-b_leg2x**2)**0.5
            l2a = math.atan2(b_leg23y, b_leg2x)
            l3a = math.atan2(b_leg23y, b_leg3x)
            b_angles = np.array(
                [l3a+4*math.pi/3, l2a, l3a, l2a+2*math.pi/3, l3a+2*math.pi/3, l2a+4*math.pi/3])
            b_coor = np.array([np.cos(b_angles)*b_r, np.sin(b_angles)*b_r])
            home_height = (abs(fixed_rods**2-(b_coor[0][0]-p_coorxy[0][0])**2-(
                b_coor[1][0]-p_coorxy[1][0])**2))**0.5 + actuator_home
            p_origin_pbasis = np.append(
                p_coorxy, np.array([np.zeros(6)]), axis=0)
            p_coor = np.append(p_coorxy, np.array(
                [np.ones(6)*home_height]), axis=0)
            legs = actuator_solving(b_coor, p_coor)
            legs = np.round(legs, 3)  # increase precison here
            previous_inputs = np.zeros((6))
            print("Starting up")
            echo()
            print("End start up")
            ini_home = "G0 X" + str(actuator_home) + " Y" + str(actuator_home) + " Z" + str(
                actuator_home) + " A" + str(actuator_home) + " B" + str(actuator_home) + " C" + str(actuator_home)
            arduino.reset_input_buffer()
            print("feedrate setting")
            # write_read("G28")
            write_read(ini_home)
            print("in waiting after start")
            print(arduino.in_waiting)
            print("homed at " + ini_home)

            print(arduino.in_waiting)
            arduino.reset_input_buffer()
            print("Flush input buffer at start up")

            state = 1
        elif num_legs == 5:
            p_angles = [[0], [2*math.pi/5], [4*math.pi/5],
                        [6*math.pi/5], [8*math.pi/5]]
            print("Platform angles", p_angles)
        elif num_legs == 4:
            p_angles = [[0], [math.pi/2], [math.pi], [3*math.pi/2]]
            print("Platform angles", p_angles)
        elif num_legs == 3:
            p_angles = [[0], [2*math.pi/3], [4*math.pi/3]]
            print("Platform angles", p_angles)
        else:
            print("leg number error")
            return

    while state == 1:
        print("Current platform coordinates")
        print(p_coor)
        print("Choose next operation:")
        print("For 6DOF input type 6dof")
        print("For G code input type gcode")
        print("To end the programme type end")
        print("To home to mid point type home")
        print("To check input buffer type buffer")
        print("For EMERGENCY STOP type stop")
        print("For cancel type cancel")
        userInput = input("input: ")

        if userInput == "6dof":

            exitClause = False

            while not exitClause:
                try:
                    x_translate = float(input("X translation absolute: "))
                    if (abs(x_translate) > range_x_translate):
                        print(
                            f"Input exceeds range of motion (\u00B1{range_x_translate}mm), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            while not exitClause:
                try:
                    y_translate = float(input("Y translation absolute: "))
                    if (abs(y_translate) > range_y_translate):
                        print(
                            f"Input exceeds range of motion (\u00B1{range_y_translate}mm), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            while not exitClause:
                try:
                    z_translate = float(input("Z translation absolute: "))
                    if (z_translate > actuator_max or z_translate < actuator_mini):
                        print(
                            f"Input exceeds range of motion (\u00B1{range_z_translate}mm), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            while not exitClause:
                try:
                    roll = (
                        float(input("Roll movement absolute in degrees: "))/180)*math.pi
                    if (abs(roll) > range_roll):
                        print(
                            f"Input exceeds range of motion (\u00B1{(range_roll/math.pi) * 180}\u00B0), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            while not exitClause:
                try:
                    pitch = (
                        float(input("Pitch movement absolute in degrees: "))/180)*math.pi
                    if (abs(pitch) > range_pitch):
                        print(
                            f"Input exceeds range of motion (\u00B1{(range_pitch/math.pi) * 180}\u00B0), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            while not exitClause:
                try:
                    yaw = (
                        float(input("Yaw movement absolute in degrees: "))/180)*math.pi
                    if (abs(yaw) > range_yaw):
                        print(
                            f"Input exceeds range of motion (\u00B1{(range_yaw/math.pi) * 180}\u00B0), please try again.")
                        continue
                    else:
                        break
                except ValueError:
                    exitClause = True
                    break
            if not exitClause:
                print("in waiting before 6dof")
                print(arduino.in_waiting)
                gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x_translate,
                      y_translate, z_translate, roll, pitch, yaw, previous_inputs)
                print("in waiting after 6dof")
                print(arduino.in_waiting)
                previous_inputs = np.array(
                    [x_translate, y_translate, z_translate, roll, pitch, yaw])

        elif userInput == "gcode":
            print("in waiting before gcode")
            print(arduino.in_waiting)
            arduino.reset_input_buffer()
            print("Flush input buffer prior to gcode movement")
            byuser = input("Type your Gcode: ")
            byuser = byuser.upper()
            write_read(byuser)
            print("in waiting after gcode")
            print(arduino.in_waiting)
            previous_inputs = np.zeros((6))
            continue
        elif userInput == "end":
            arduino.reset_input_buffer()
            write_read("M18")
            print("in waiting")
            print(arduino.in_waiting)
            arduino.reset_input_buffer()
            print("in waiting2")
            print(arduino.in_waiting)
            arduino.close()
            state = 0
            break
        elif userInput == "buffer":
            print("in waiting")
            print(arduino.in_waiting)
        elif userInput == "home":
            try:
                print("Homing platform")
                home(p_coor, p_origin_pbasis,
                     p_coor_pbasis, b_coor, previous_inputs)
                time.sleep(0.5)
                previous_inputs = np.zeros((6))
            except:
                pass
            continue
        elif userInput == "stop":
            write_read("M112")
        elif userInput == "cancel":
            write_read("M410")
        if userInput == "flex":
            casualflex(b_coor, p_coor)
            # home(p_coor, p_origin_pbasis,p_coor_pbasis, b_coor, previous_inputs)
            time.sleep(2.5)
            arduino.reset_input_buffer()
            time.sleep(0.2)
            recasualflex(b_coor, p_coor)
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            time.sleep(1)

            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, math.pi/6, 0, 0])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, -math.pi/6, 0, 0])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            time.sleep(1)
            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, 0, math.pi/6, 0])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, 0, -math.pi/6, 0])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            time.sleep(1)
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, 0, 0, math.pi/6])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            gcode(p_coor, p_origin_pbasis, p_coor_pbasis,
                  b_coor, 0, 0, 0, 0, 0, 0, previous_inputs)
            previous_inputs = np.array([0, 0, 0, 0, 0, -math.pi/6])
            home(p_coor, p_origin_pbasis,
                 p_coor_pbasis, b_coor, previous_inputs)
            previous_inputs = np.zeros((6))

        else:
            continue

# For Match-case statement only supported in Python 3.10 and newer

        # match input:
        #     case '6dof':
        #         previous_inputs = np.array(
        #             [x_translate, y_translate, z_translate, roll, pitch, yaw])
        #         x_translate = float(input("X translation absolute: "))
        #         y_translate = float(input("Y translation absolute: "))
        #         z_translate = float(input("Z translation absolute: "))
        #         roll = (
        #             float(input("Roll movement absolute in degrees: "))/180)*math.pi
        #         pitch = (
        #             float(input("Pitch movement absolute in degrees: "))/180)*math.pi
        #         yaw = (float(input("Yaw movement absolute in degrees: "))/180)*math.pi
        #         print("in waiting before 6dof")
        #         print(arduino.in_waiting)
        #         gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x_translate,
        #               y_translate, z_translate, roll, pitch, yaw, previous_inputs)
        #         print("in waiting after 6dof")
        #         print(arduino.in_waiting)
        #         continue

        #     case 'gcode':
        #         print("in waiting before gcode")
        #         print(arduino.in_waiting)
        #         arduino.reset_input_buffer()
        #         print("Flush input buffer prior to gcode movement")
        #         write_read(input("Type your Gcode: "))
        #         print("in waiting after 6dof")
        #         print(arduino.in_waiting)
        #         continue

        #     case 'end':
        #         write_read("M18")
        #         time.sleep(1)
        #         print("in waiting")
        #         print(arduino.in_waiting)
        #         arduino.reset_input_buffer()
        #         print("in waiting2")
        #         print(arduino.in_waiting)
        #         arduino.close()
        #         state = 0
        #         break

        #     case 'buffer':
        #         print("in waiting")
        #         print(arduino.in_waiting)

        #     case 'home':
        #         previous_inputs = np.array(
        #             [x_translate, y_translate, z_translate, roll, pitch, yaw])
        #         print("Homing platform")
        #         home(p_coor, p_origin_pbasis,
        #              p_coor_pbasis, b_coor, previous_inputs)
        #         time.sleep(0.5)
        #         print(arduino.out_waiting)

        #     case _:
        #         continue


# start code form here
b_r = 125  # float(input("Base radius: "))
p_r = 75  # float(input("Platform radius: "))
actuator_mini = 0  # float(input("Actuator unextended: "))
actuator_max = 300  # float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2) + actuator_mini
fixed_rods = 210  # float(input("Fixed rod lengths: "))
actuator_Precision = 3  # Number of DP for actuator length
max_change_per_slice = 1  # Change resolution of movements here
# Minimum slices per movement (can be removed if not needed)
minimum_slice_per_movement = 10
range_x_translate = 150
range_y_translate = 150
range_z_translate = (actuator_max-actuator_mini)/2
range_roll = 0.524
range_pitch = 0.524
range_yaw = 0.524


menu()
