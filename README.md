# Using

## 1. Install Python Dependencies

Install ur_rtde, pygame and numpy for the program functionality

```
pip install -r requirements.txt
```
## or 

### **RTDE**: &nbsp; Provides real-time data exchange with universal robot
```
pip install ur_rtde
```

### **Pygame**:&nbsp; Provides joystick driver

```
pip install pygame
```
### **Numpy**:

```
pip install numpy
```

## 2. Safety Configuration

The satefy configuration is not a requirement to run the program but for safety, we provide some safety setup. The default config settings is set to false. All parameters are specified in robot_config.yaml file. 

Controlling the robot using joystick is base on transforming the values of _[x, y, z, rx, ry, rz]_ and _[j0, j1, j2, j3, j4, j5]_ of the robot for motions linear in cartesian and joint space respectively.


* Joint limit <br />
The maximum rotation of the robot joint given from the universal robot specification is approximately from -2π to 2π.

* TCP limit <br />
To prevent the robot TCP from exceeding its reach which is defined by an equation of a sphere whose displacement from the center is not greater than the the maximum radius given by the robot specification (eg. UR3 max radius = 500mm). <br />
Control the robot (cartesian) to the maximum reach in either of the axes (x, y or z) and record the value of x, y and z on the teach pendent or ursim and sum their squares _(max_dsq = x^2 + y^2 + z^2)_ as show in fig[A]. <br />
Set feature as shown in the figure to Base before reading any values for configuration. <br />
Also, to prevent the robot from colliding with itself, we create an imaginary cylinder define by a circle as show in fig[B]. Move the robot to the minimun reach and record the value of x and y and sum the their squares _(min_dsq = x^2 + y^2)_. <br />
All values are in meters. <br />
Set _displacement_sq = [min_sq, max_sq]_ in the robot_config.yaml file.

* Mount surface <br />
From figure the below, our assumption for the the mount surface is rectangular surface. we define the surface as _mount_surf = [x_min, x_max, y_min, y_max]_ (see fig[D]). Also, take note of the collision with the mount surface (see fig[C]) _mount_surf = [x_min, x_max, y_min, y_max, z_min]_. <br />
Not every mounting surface of the robot base is large enough for consideration, so default _mount_surf = [0, 0, 0, 0, 0]_. <br /> <br />
![safety config steps](https://github.com/sadichel/URJoystickControl/blob/f7052178ef5a7b8d073cd8600b8311b130b951e9/img/robot_config_steps.png?raw=true) 

* Base length <br />
Base length is a important parameter in the robot_config.yaml file for center correction. [See this for reference](https://www.universal-robots.com/media/1803022/5ework.png?width=704&height=731). For UR3 and UR5, we have base length of 152mm and 163mm respectively (UR3: _base_length = 0.152_).

## 3. Controlling Robot
* Joystick control mapping <br /> <br />
![Joystick control map](https://github.com/sadichel/URJoystickControl/blob/main/img/joystick_control_map.png?raw=true)


* Default Orientation <br /> <br />
![Defaul Orientation](https://github.com/sadichel/URJoystickControl/blob/main/img/default_orientation.png?raw=true)


