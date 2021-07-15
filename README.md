# stewart-platform

## Python script

### Variable list for a Hexapod

`p_coor` – The coordinates of the platform joints taking XYZ=0 at the center of breadboard. Changes with movement. Shape is 3 by 6

`p_coor_home` – The coordinates of the platform joints and its home position, middle of actuator, taking XYZ=0 at the center of breadboard. A constant varible of shape 3 by 6

`p_coor_pbasis` – The coordinates of the platform joints taking XYZ=0 at the center of platform. A constant varible of shape 3 by 6

`b_coor` – The coordinates of the actuators base taking XYZ=0 at the center of breadboard. A constant varible of shape 3 by 6. With the third row being all zeros, z=0.

`b_r` - Base radius of actuator legs, float. 

`p_r` - Platform radius, float

`actuator_mini` - z value of the unextended actuator, float

`actuator_max` - z value of the fully extended actuator, float

`actuator_home` - mid point of actuator working length. Also the home position of hexapod

`fixed_rods` - lenght of the fixed rods, including joint. 

`actuator_Precision` - decimal place to round off gcode to. default units is mm

`max_change_per_slice` - divisor for slicing number operation. Equals to maximum movement in mm per slice. 

`minimum_slice_per_movement` - default minimum number of slices for any movement 

`range_x_translate` - range of x translate in mm

`range_y_translate` - range of x translate in mm

`range_z_translate` = (actuator_max-actuator_mini)/2 - range of movement from the mid point. 

`roll` - rotation along the x axis

`pitch` - rotation along the y axis

`yaw` - rotation along the  axis

`range_roll` - range of roll in radians

`range_pitch` - range of pitch in radians

`range_yaw` - range of yaw in radians









Flow of functions. 

Runs lines 540 to 556
    Takes user defined configuration and settings.

`menu()`
    Has 2 parts
    `state=0`
        Sets all 6dof inputs to zero and obtains the number of legs in the config. 
        Takes user defined configuration and runs the initial calculation of `p_coor`, `p_coor_home`, `p_coor_pbasis`, `b_coor` according to the number of legs and platform angles and base angles. 
        Creates a 1 by 6 array of zeros called previous_inputs
        Pings Marlin using `echo()` function to return all start up messages.
        Runs G28 to home all actuators to the endstops. 
        Raises all actuators to the home position. 
        Flushes input buffer to ensure there is no input issues later.
        Invokes state=1 
    `state=1`
        Prints the current platform coordinates, `p_coor` 
        Ask user for the operation they want. 
            6dof 
                Takes inputs of the 6dof, compares it with the range and limits of 6dof defined by user previously. If all values are valid, passes them to function `gcode()`. 
                After running `gcode()`, store the inputs as previous inputs. 
            gcode
                flushes input buffer 
                Takes a user input string of Gcode and uppercases it. 
                Sends the string to Marlin via arduino.write.read
                resets the previous input to zeros. 
                ***Using gocde function randomly may cause issues with movement. Should only be used when at home position 
            end 
                Flush input buffer 
                Run Gcode M18 to stop all motors, stops motor holding 
                Flush again 
                Close port.
            buffer
                Not important, just return the input buffer for debugging issues 
            home
                returns the platform to the home position at mid pt of actuators. 
                Calles function `home()` that takes the current p_coor and previous_inputs and calculate a smooth sliced path back to home position.
            stop 
                runs Gcode M112 
            cancel 
                run Gcode M410 
            flex 
                homes the platform, 
                runs `flex()`, a predetermined circular xyz translation routine.
                runs `reflex()`, a reverse of `flex()`
                homes the platform. 
            casualflex 
                moves the platfrom to x=30 and pitch by -30 degrees.
                runs `rotatingflex()`, a predetermined circular rotation routine.
                homes platform 

`write_read(x)`
    argument x is the gcode string that will be sent to Marlin. The function adds a \r\n to the end of the string to be read by Marlin. Marlin returns a string via `data = arduino.readline()`
    
`echo()`
    Pause for 4 seconds to allow Marlin to buffer and start up, prevents user from pinging Marlin too quickly and causing input issues. Pings Marlin 23 time to get all start up messages.  

flex()
    Divides 360degrees by 180 into segments of 2degree and iterates through from 0 to 180.  In each loop, the angle along the path is calculated as `angle`, an array of ones is created for each x,y,z. The sin and cos proportion are multiplied by 30 to form the x and y translation along a circular path of 60mm .
    In each iteration, the platform increase by z 0.5.
    These x y z arrays are concatenated to form a translation matrix which is added to `p_coor_home`. This gives `cir_p_coor`, the platform coordinates along the circular path.
    `cir_p_coor` is sent to `actuator_solving()` and the actuator heights are returned as an array and rounded down to a decimal point stated by `actuator_Precision` 
    the actautor heights are extracted from the array and placed in a gcode string as `output`
    `output` is sent to `write_read()`

`reflex()`
    does `flex()` in reverse

`rotatingflex()`
    Divides 360degrees by 180 into segments of 2degree and iterates through from 0 to 180.  In each loop, the angle along the path is calculated as `angle`. math.cos and math.sin is used to calculate the position along circular path. Uses the values of the sin and cos to calculate the proportion of change along the path.
    The sin and cos proportion are multiplied by 30 to form the x and y translation along a circular path of 30mm . 
    The sin and cos proportion are multiplied by pitch and roll of -30 and 30 degrees to form a rotation that alwasy tracks the center along its circular path. 
    The pitch and roll are applied to the `p_coor_pbasis` and the the x and y translation are added to form the coordinates of the platform along the routine. 
 

rotation_simple(psi, theta, phi)
    Takes in the 3 angles of rotation. Psi for yaw, theta for pitch and phi for roll. Applies the 3 angles to the combined rotation matrix. This matrix is then matrix multiplied later. 

actuator_solving(b_coor, p_coor)
    Takes the x and y values of each platform joint, `b_coor` and `fixed_rods` to calculate the z length of the angled rods/fixed rods. 
    This z length of the angled rod is deducted from the z height of the platform to find the actuator length. 
    The actuator lengths are placed in an array and returned as leggy. 
 

slicing_number_generator(start_pose, end_pose, b_coor)
    Takes the `start_pose` and solves for the actuator lengths. (Initial position)
    Takes the `end_pose` and solves for its actuator length. (Final position)
    Compares the starting and ending position of each actuator and finds the largest change. The largest change is divided by the max_change_per_slice to obtain a slicing_number. This is currently set as 1, so the largest movement per slice is 1mm. 
    Next compare the slicing_number and minimum_slice_per_movement. If it is smaller than minimum_slice_per_movement, return the minimum_slice_per_movement. Else slicing number is used.

home(p_coor, previous_inputs)
    does what `gcode()` does but final position is always `p_coor_home` by using 0 for all new inputs. 
    resets `previous_inputs` to zeros outside the function.
	

gcode(p_coor, x, y, z, roll, pitch, yaw, previous_inputs)
    Applies the rotation matrix, `rotation_simple()`, on the p_coor_pbasis. Has been be done on platform basis so that the rotation pivot is the center of platform. The translation values are added to the result to form the final rotated and translated platform coordinates. 
    The slicing number is generated by `slicing_number_generator()`.
    The slicing operation take the new 6dof inputs and the previous inputs and plots out the motion from start pose to final pose. 
        It does so by calculating the intermediate 6dof values from start to final position. 
        It uses the intermediate valeus and applies the rotation matrix and translation to plot out the `intermediate p_coor`.
        It then uses `actuator_solving()` to solve the actuator postion for each intermediate slice and sends it to Marlin. 
    Runs the final position 3 extra times to ensure it reaches the position. Not essential
    outside of the function, the inputs are saved under previosu inputs
`



