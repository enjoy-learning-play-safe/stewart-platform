import numpy as np
import time
import math
from echo import echo
import actuator_solving
import write_read
from home import home
from gcode import gcode
from config import *
import casualflex
import recasualflex
import rotatingflex



def menu():
    state = 0
    while state == 0:

        x_translate = 0
        y_translate = 0
        z_translate = 0
        roll = 0
        pitch = 0
        yaw = 0
        previous_inputs = np.zeros((6))
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
            p_coor_home = np.append(p_coorxy, np.array(
                [np.ones(6)*home_height]), axis=0)
            p_coor = np.append(p_coorxy, np.array(
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
            p_coor = home(p_coor,
                           previous_inputs)
            previous_inputs = np.zeros((6))
            casualflex()
            time.sleep(2.5)
            arduino.reset_input_buffer()
            time.sleep(0.2)
            p_coor = recasualflex()
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

menu()