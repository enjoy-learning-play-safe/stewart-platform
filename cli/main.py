from menu import menu

# start code form here
b_r = 125  # float(input("Base radius: "))
p_r = 75  # float(input("Platform radius: "))
actuator_mini = 0  # float(input("Actuator unextended: "))
actuator_max = 390  # float(input("Actuator fully extended: "))
actuator_home = ((actuator_max-actuator_mini)/2) + actuator_mini
fixed_rods = 209  # float(input("Fixed rod lengths: "))


menu(b_r, p_r, actuator_mini, actuator_max, actuator_home, fixed_rods)
