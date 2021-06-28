
import numpy as np

def actuator_solving(b_coor, p_coor, fixed_rods):
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
