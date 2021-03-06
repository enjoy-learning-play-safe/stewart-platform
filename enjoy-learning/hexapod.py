import math
import numpy as np
import serial
import time

arduino = serial.Serial(port='COM4', baudrate=250000, timeout=0.2)


def write_read(x):
    x = str(x) + "\r\n"
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.03)
    data = arduino.readline()
    print(data)
    return


def echo():
    time.sleep(4)
    ping = 23
    time.sleep(0.02)
    while ping > 0:
        write_read("G90")
        ping = ping - 1


def flex():
    index = 180
    angle = 0
    cir_p_coor = p_coor_home
    n = 0
    while index > 0:
        n = n+1
        change = math.pi/90
        angle = angle + change
        x_coor = np.array([np.ones((6))])* math.cos(angle)*60
        y_coor = np.array([np.ones((6))])* math.sin(angle)*60
        z_coor = np.array([np.ones((6))])* 0.5*n
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


def reflex():
    index = 180
    angle = 0
    cir_p_coor = p_coor_home
    n = 0
    while index > 0:
        n = n+1
        change = math.pi/90
        angle = angle - change
        x_coor = np.array([np.ones((6))])* math.cos(angle)*60
        y_coor = np.array([np.ones((6))])* math.sin(angle)*60
        z_coor = np.array([np.ones((6))])*90 - 0.5*n
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
    print("done reverse")
    return p_coor_home

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
        rott = np.matmul(rotation_simple(0, pitch, roll),p_coor_pbasis)
        p_coor = np.array([rott[0] + x_coor, rott[1] +y_coor, rott[2]]) - p_coor_pbasis + p_coor_home
        legs = actuator_solving(p_coor)
        legs = np.round(legs, actuator_Precision) 
        output = "G0 X" + str(legs[0]) + " Y" + str(legs[1]) + " Z" + str(legs[2]) + " A" + str(legs[3]) + " B" + str(legs[4]) + " C" + str(legs[5])
        write_read(output)
        time.sleep(0.01)
        # print(np.round(p_coor,2))
        print(output)
        index = index -1
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


def actuator_solving(p_coor):
    leg1 = p_coor[2][0] - (abs(fixed_rods**2 - (p_coor[0][0] -
                           b_coor[0][0])**2 - (p_coor[1][0] - b_coor[1][0])**2))**0.5
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


def slicing_number_generator(start_pose, end_pose):
    # p_coor is already the previous platform coordinate before movement. no need to solve it again
    previous_legs = actuator_solving(start_pose)
    previous_legs = np.round(previous_legs, actuator_Precision)
    final_legs = actuator_solving(end_pose)
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
        rotated = np.matmul(rotation_simple(intermediate_yaw, intermediate_pitch, intermediate_roll),p_coor_pbasis)
            
        intermediate_p_coor = np.array([rotated[0] + intermediate_x, rotated[1] +
                                       intermediate_y, rotated[2] + intermediate_z]) - p_coor_pbasis + p_coor_home
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


def gcode(p_coor, x, y, z, roll, pitch, yaw, previous_inputs):
    start_pose = p_coor
    rott = np.matmul(rotation_simple(yaw, pitch, roll),p_coor_pbasis)
    p_coor = np.array([rott[0] + x, rott[1] + y, rott[2]+z]
                      ) - p_coor_pbasis + p_coor_home
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
            intermediate_yaw, intermediate_pitch, intermediate_roll),p_coor_pbasis)
        intermediate_p_coor = np.array([rotated[0] + intermediate_x, rotated[1] +
                                       intermediate_y, rotated[2]+intermediate_z]) - p_coor_pbasis + p_coor_home
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


def menu():
    state = 0
    while state == 0:
        x_translate = y_translate = z_translate = roll = pitch =yaw = 0
        previous_inputs = np.zeros((6))
        num_legs = 6      # num_legs= int(input("Number of legs: "))
        if num_legs == 6:
            p_angles = np.array(
                [0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3])
            p_coorxy = np.array([np.cos(p_angles)*p_r, np.sin(p_angles)*p_r])
            global p_coor_pbasis
            p_coor_pbasis = np.append(p_coorxy, np.array([np.zeros(6)]), axis=0)
            b_leg2x = p_coorxy[0][1]
            b_leg3x = p_coorxy[0][2]
            b_leg23y = (b_r**2-b_leg2x**2)**0.5
            l2a = math.atan2(b_leg23y, b_leg2x)
            l3a = math.atan2(b_leg23y, b_leg3x)
            b_angles = np.array(
                [l3a+4*math.pi/3, l2a, l3a, l2a+2*math.pi/3, l3a+2*math.pi/3, l2a+4*math.pi/3])
            global b_coor
            b_coor = np.array([np.cos(b_angles)*b_r, np.sin(b_angles)*b_r])
            home_height = (abs(fixed_rods**2-(b_coor[0][0]-p_coorxy[0][0])**2-(
                b_coor[1][0]-p_coorxy[1][0])**2))**0.5 + actuator_home                
            p_coor = np.append(p_coorxy, np.array(
                [np.ones(6)*home_height]), axis=0)
            global p_coor_home
            p_coor_home = np.append(p_coorxy, np.array(
                [np.ones(6)*home_height]), axis=0)
            legs = actuator_solving(p_coor)
            legs = np.round(legs, actuator_Precision)  # increase precison here
            previous_inputs = np.zeros((6))
            print("Starting up")
            echo()
            print("End start up")
            ini_home = "G0 X" + str(actuator_home) + " Y" + str(actuator_home) + " Z" + str(
                actuator_home) + " A" + str(actuator_home) + " B" + str(actuator_home) + " C" + str(actuator_home)
            arduino.reset_input_buffer()
            write_read("G28")
            time.sleep(3)
            write_read(ini_home)
            print("in waiting after start")
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
        print(np.round(p_coor, 3))
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
                    if (abs(z_translate) > range_z_translate):
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
                p_coor = gcode(p_coor,
                               x_translate, y_translate, z_translate, roll, pitch, yaw, previous_inputs)
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
                p_coor = home(p_coor,previous_inputs)
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
            p_coor = home(p_coor,previous_inputs)
            previous_inputs = np.zeros((6))
            flex()
            time.sleep(2.5)
            arduino.reset_input_buffer()
            time.sleep(0.2)
            p_coor = reflex()
            p_coor = home(p_coor,
                          previous_inputs)
            previous_inputs = np.zeros((6))
            time.sleep(2)

        if userInput == "casualflex":
            p_coor = gcode(p_coor, 30,0, 0, 0, -math.pi/6, 0, previous_inputs)
            previous_inputs = np.array([30, 0, 0, 0, -math.pi/6, 0])  
            time.sleep(1)
            rotatingflex()
            time.sleep(2)
            p_coor =home(p_coor, previous_inputs)
            previous_inputs = np.zeros((6))
            x_translate = pitch =0

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
        #         gcode(p_coor, p_coor_pbasis, p_coor_pbasis, b_coor, x_translate,
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
        #         p_coor =home(p_coor, p_coor_pbasis,
        #              p_coor_pbasis, b_coor, previous_inputs)
        #         time.sleep(0.5)
        #         print(arduino.out_waiting)

        #     case _:
        #         continue


# start code form here
b_r = 123.7  # float(input("Base radius: "))
p_r = 75  # float(input("Platform radius: "))
actuator_mini = 0  # float(input("Actuator unextended: "))
actuator_max = 300  # float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2) + actuator_mini
fixed_rods = 210  # float(input("Fixed rod lengths: "))
actuator_Precision = 3  # Number of DP for actuator length
max_change_per_slice = 1  # Change resolution of movements here
minimum_slice_per_movement = 10
range_x_translate = 100
range_y_translate = 100
range_z_translate = (actuator_max-actuator_mini)/2
range_roll = 0.524
range_pitch = 0.524
range_yaw = 0.524


menu()
