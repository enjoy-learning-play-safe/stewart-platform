import math
import numpy as np
import serial
import time

arduino = serial.Serial(port='COM5', baudrate=250000, timeout=0.02)

def write_read(x):
    x = str(x) + "\r"
    arduino.write(bytes(x, 'utf-8'))
    #arduino.writelines(bytes(x, 'utf-8'))
    time.sleep(0.02)
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


def menu():
    state = 0
    while state==0:

        x_translate=0
        y_translate=0
        z_translate=0
        roll=0
        pitch=0
        yaw=0
        num_legs=6      # num_legs= int(input("Number of legs: "))
        if num_legs== 6:
            p_angles= np.array([0,math.pi/3,2*math.pi/3,math.pi,4*math.pi/3,5*math.pi/3])
            p_coorxy= np.array([np.cos(p_angles)*p_r, np.sin(p_angles)*p_r])
            p_coor_pbasis= np.append(p_coorxy, np.array([np.zeros(6)]), axis = 0)
            b_leg2x= p_coorxy[0][1]
            b_leg3x= p_coorxy[0][2] 
            b_leg23y= (b_r**2-b_leg2x**2)**0.5
            l2a= math.atan2(b_leg23y,b_leg2x)
            l3a= math.atan2(b_leg23y,b_leg3x)
            b_angles= np.array([ l3a+4*math.pi/3,l2a,l3a,l2a+2*math.pi/3,l3a+2*math.pi/3,l2a+4*math.pi/3])
            b_coor= np.array([np.cos(b_angles)*b_r, np.sin(b_angles)*b_r])  
            home_height=(abs(fixed_rods**2-(b_coor[0][0]-p_coorxy[0][0])**2-(b_coor[1][0]-p_coorxy[1][0])**2))**0.5 +actuator_home
            p_origin_pbasis = np.append(p_coorxy, np.array([np.zeros(6)]), axis = 0)
            p_coor= np.append(p_coorxy, np.array([np.ones(6)*home_height]), axis = 0)
            legs= actuator_solving(b_coor, p_coor)
            legs = np.round(legs,3)                     #increase precison here 
            previous_inputs= np.zeros((6)) 
            print("Starting up")
            echo()
            print("End start up")
            ini_home = "G0 X" + str(actuator_home) + " Y" + str(actuator_home) + " Z" + str(
                actuator_home) + " A" + str(actuator_home) + " B" + str(actuator_home) + " C" + str(actuator_home) + "F1600"
            arduino.reset_input_buffer()
            print("feedrate setting")
            print("Started Homing sequence")
            write_read("G28")
            time.sleep(2)
            print("Moving to origin")
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
        userInput = input("input: ")

        if userInput == "6dof":
            previous_inputs = np.array([x_translate, y_translate, z_translate, roll, pitch, yaw])
            try: 
                x_translate = float(input("X translation absolute: "))
                y_translate = float(input("Y translation absolute: "))
                z_translate = float(input("Z translation absolute: "))
                roll = (float(input("Roll movement absolute in degrees: "))/180)*math.pi
                pitch = (float(input("Pitch movement absolute in degrees: "))/180)*math.pi
                yaw = (float(input("Yaw movement absolute in degrees: "))/180)*math.pi
                print("in waiting before 6dof")
                print(arduino.in_waiting)
                gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x_translate,y_translate, z_translate, roll, pitch, yaw, previous_inputs)
                print("in waiting after 6dof")
                print(arduino.in_waiting)
                previous_inputs = np.array([x_translate,y_translate,z_translate,roll,pitch,yaw])
            except:
                pass
            continue

        elif userInput == "gcode":  
            print("in waiting before gcode")
            print(arduino.in_waiting)
            arduino.reset_input_buffer()
            print("Flush input buffer prior to gcode movement")
            byuser= input("Type your Gcode: ")
            byuser = byuser.upper()
            write_read(byuser)
            print("in waiting after 6dof")
            print(arduino.in_waiting)
            previous_inputs= np.zeros((6))
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
            print("Homing platform")
            home(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, previous_inputs)
            time.sleep(0.5)
            previous_inputs= np.zeros((6))
        elif userInput == "stop":
            arduino.reset_input_buffer()
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
actuator_max = 390  # float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2) + actuator_mini
fixed_rods = 209  # float(input("Fixed rod lengths: "))


menu()
