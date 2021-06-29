
from cli.write_read import write_read
import numpy as np
import time
import math
from echo import echo
from actuator_solving import actuator_solving
from write_read import *
from home import home
from gcode import gcode


def menu(b_r, p_r, actuator_mini, actuator_max, actuator_home, fixed_rods):
    state = 0
    while state == 0:

        x_translate = 0
        y_translate = 0
        z_translate = 0
        roll = 0
        pitch = 0
        yaw = 0
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
            legs = actuator_solving(b_coor, p_coor, fixed_rods)
            legs = np.round(legs, 3)  # increase precison here
            previous_inputs = np.zeros((6))
            print("Starting up")
            echo()
            print("End start up")
            ini_home = "G0 X" + str(actuator_home) + " Y" + str(actuator_home) + " Z" + str(
                actuator_home) + " A" + str(actuator_home) + " B" + str(actuator_home) + " C" + str(actuator_home) + " F80"
            arduino.reset_input_buffer()
            print("feedrate setting")
            write_read(ini_home)
            time.sleep(3)
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
            previous_inputs = np.array(
                [x_translate, y_translate, z_translate, roll, pitch, yaw])
            try:
                x_translate = float(input("X translation absolute: "))
                y_translate = float(input("Y translation absolute: "))
                z_translate = float(input("Z translation absolute: "))
                roll = (
                    float(input("Roll movement absolute in degrees: "))/180)*math.pi
                pitch = (
                    float(input("Pitch movement absolute in degrees: "))/180)*math.pi
                yaw = (float(input("Yaw movement absolute in degrees: "))/180)*math.pi
                print("in waiting before 6dof")
                print(arduino.in_waiting)
                gcode(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x_translate,
                      y_translate, z_translate, roll, pitch, yaw, previous_inputs)
                print("in waiting after 6dof")
                print(arduino.in_waiting)
                previous_inputs = np.array(
                    [x_translate, y_translate, z_translate, roll, pitch, yaw])
            except:
                pass
            continue

        elif userInput == "gcode":
            print("in waiting before gcode")
            print(arduino.in_waiting)
            arduino.reset_input_buffer()
            print("Flush input buffer prior to gcode movement")
            byuser = input("Type your Gcode: ")
            byuser = byuser.upper()
            write_read(byuser)
            print("in waiting after 6dof")
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
            print("Homing platform")
            home(p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, previous_inputs)
            time.sleep(0.5)
            previous_inputs = np.zeros((6))
        elif userInput == "stop":
            arduino.reset_input_buffer()
        else:
            continue
