import math 
import numpy as np


def rotation_simple(psi, theta, phi):
    cpsi= math.cos(psi)
    ctheta = math.cos(theta)
    cphi=math.cos(phi)
    spsi= math.sin(psi)
    stheta = math.sin(theta)
    sphi=math.sin(phi)
    rota= np.array([[cpsi*ctheta, (cpsi*stheta*sphi-spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)],[spsi*ctheta, (cpsi*cphi+spsi*stheta*sphi), (spsi*stheta*cphi-cpsi*sphi)],[-stheta, ctheta*sphi, ctheta*cphi]]) 
    # [cpsi*ctheta, (cpsi*stheta*sphi-spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)]
    # [spsi*ctheta, (cpsi*cphi+spsi*stheta*sphi), (spsi*stheta*cphi-cpsi*sphi)]
    # [-stheta, ctheta*sphi, ctheta*cphi]
    return rota

def actuator_solving(b_coor,p_coor):
    leg1= p_coor[2][0]-(abs(fixed_rods**2-(p_coor[0][0]-b_coor[0][0])**2-(p_coor[1][0]-b_coor[1][0])**2))**0.5         #not ready for non hexapod
    leg2= p_coor[2][1]-(abs(fixed_rods**2-(p_coor[0][1]-b_coor[0][1])**2-(p_coor[1][1]-b_coor[1][1])**2))**0.5 
    leg3= p_coor[2][2]-(abs(fixed_rods**2-(p_coor[0][2]-b_coor[0][2])**2-(p_coor[1][2]-b_coor[1][2])**2))**0.5 
    leg4= p_coor[2][3]-(abs(fixed_rods**2-(p_coor[0][3]-b_coor[0][3])**2-(p_coor[1][3]-b_coor[1][3])**2))**0.5 
    leg5= p_coor[2][4]-(abs(fixed_rods**2-(p_coor[0][4]-b_coor[0][4])**2-(p_coor[1][4]-b_coor[1][4])**2))**0.5 
    leg6= p_coor[2][5]-(abs(fixed_rods**2-(p_coor[0][5]-b_coor[0][5])**2-(p_coor[1][5]-b_coor[1][5])**2))**0.5 
    leggy = np.array([leg1,leg2,leg3,leg4,leg5,leg6]) 
    return leggy
    


def gcode( p_coor, p_origin_pbasis, p_coor_pbasis, b_coor, x,y,z,roll,pitch,yaw):
    slicing_number = 20   #tune movement
    increment =slicing_number
    n=0
    while slicing_number >0:
        n=n+1
        inc_x= (x/increment)*n
        inc_y= (y/increment)*n
        inc_z= (z/increment)*n
        inc_roll= (roll/increment)*n
        inc_pitch= (pitch/increment)*n
        inc_yaw= (yaw/increment)*n   
        rotated = np.matmul(rotation_simple(inc_roll,inc_pitch,inc_yaw), p_coor_pbasis)     
        final_p_coor = np.array([rotated[0]+inc_x, rotated[1]+inc_y, rotated[2]+inc_z])  -p_origin_pbasis + p_coor
        legs= actuator_solving(b_coor, final_p_coor)
        legs = np.round(legs,2)                     #increase precison here 
        print("G0 X"+str(legs[0])+ " Y"+str(legs[1])+" Z"+str(legs[2])+" A"+str(legs[3])+" B"+str(legs[4])+" C"+str(legs[5])) #not ready for non hexapod
        slicing_number=slicing_number-1
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
            print(p_coor)
            state=1
        elif num_legs == 5:
            p_angles= [[0],[2*math.pi/5],[4*math.pi/5],[6*math.pi/5],[8*math.pi/5]]
            print("Platform angles", p_angles)
        elif num_legs == 4:
            p_angles= [[0],[math.pi/2],[math.pi],[3*math.pi/2]]
            print("Platform angles", p_angles)
        elif num_legs == 3:
            p_angles= [[0],[2*math.pi/3],[4*math.pi/3]]
            print("Platform angles", p_angles)
        else:
            print("leg number error")
            return
        print("end state 0")
    
    while state ==1:
        print("state started, give inputs")
        print("Current platform coordinates")
        print(p_coor)
        
        
        
        x_translate= float(input("X translation absolute: "))
        y_translate= float(input("Y translation absolute: "))
        z_translate= float(input("Z translation absolute: "))
        roll= (float(input("Roll movement absolute in degrees: "))/180)*math.pi  
        pitch= (float(input("Pitch movement absolutein degrees: "))/180)*math.pi 
        yaw= (float(input("Yaw movement absolutein degrees: "))/180)*math.pi   
        
        gcode(p_coor, p_origin_pbasis,p_coor_pbasis,b_coor,x_translate,y_translate,z_translate,roll,pitch,yaw)
        if input("continue? yes or no ")=="yes":
            state=1
        else:
            state=0 

        



# start code form here
b_r= 150 #float(input("Base radius: "))
p_r= 100 #float(input("Platform radius: "))
actuator_mini = 200 #float(input("Actuator unextended: "))
actuator_max =500 #float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2)+actuator_mini
fixed_rods= 200  #float(input("Fixed rod lengths: "))
        


menu()
        
        
        
















