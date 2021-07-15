import serial

# start code form here
b_r = 123.7  # float(input("Base radius: "))
p_r = 75  # float(input("Platform radius: "))
actuator_mini = 0  # float(input("Actuator unextended: "))
actuator_max = 300  # float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2) + actuator_mini
fixed_rods = 210  # float(input("Fixed rod lengths: "))
actuator_Precision = 3  # Number of DP for actuator length
max_change_per_slice = 1  # Change resolution of movements here
# Minimum slices per movement (can be removed if not needed)
minimum_slice_per_movement = 10
range_x_translate = 100
range_y_translate = 100
range_z_translate = (actuator_max-actuator_mini)/2
range_roll = 0.524
range_pitch = 0.524
range_yaw = 0.524
p_coor_pbasis = 0
p_origin_pbasis = 0
p_coor_home = 0


arduino = serial.Serial(port='COM5', baudrate=250000, timeout=0.02)

def coor_ini(p_coor_pbasis, p_origin_pbasis, p_coor_home):
    p_coor_pbasis = p_coor_pbasis
    p_origin_pbasis = p_origin_pbasis
    p_coor_home = p_coor_home

