# stewart-platform

## Python script

### Variable list for a Hexapod

`p_coor` – The coordinates of the platform joints taking XYZ=0 at the center of breadboard. Changes with movement. Shape is 3 by 6

`p_coor_home` – The coordinates of the platform joints and its home position, middle of actuator, taking XYZ=0 at the center of breadboard. Shape is 3 by 6

`p_coor_pbasis` – The coordinates of the platform joints taking XYZ=0 at the center of platform. Shape is 3 by 6

`p_origin_pbasis` – The coordinates of the platform joints taking XYZ=0 at the center of platform. Shape is 3 by 6

_Todo: p_coor_pbasis and p_origin_pbasis are the same and we can combine them to p_origin_pbasis_

`b_coor` – The coordinates of the actuators base taking XYZ=0 at the center of breadboard. Shape is 3 by 6. With the third row being all zeros.
