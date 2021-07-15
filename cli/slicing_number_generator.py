import actuator_solving
import numpy as np
from config import *
import math

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