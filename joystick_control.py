import math
import pygame
import controllers
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time
from utils import *
import numpy as np

np.set_printoptions(precision=4, floatmode='fixed')
pygame.init()

# Initialize Pygame Joystick and select controller
try:
    id = 0
    joystick = pygame.joystick.Joystick(id)
    joystick.init()
    print(joystick.get_name())
    if joystick.get_numaxes() in [4, 6]:
        controller = find_controller(
            joystick.get_name(), controllers.CONTROLLERS)

    joystick = JoystickHandler(joystick)

except Exception as e:
    print(e)
    quit()

rtde_r = RTDEReceiveInterface('192.168.3.196')
rtde_c = RTDEControlInterface('192.168.3.196')

# rtde_r = RTDEReceiveInterface('localhost')
# rtde_c = RTDEControlInterface('localhost')

print('Connection Established with Univeral Robot')

acceleration = 0.5
velocity = 0.25
dt = 1.0/500  # 500Hz transmission rate
lookahead_time = 0.1
gain = 300
delta = 0.00025
theta = 0.00025*4
max_delta = 0.001
max_theta = 0.001
rate = 0.00005
# rate = 500 

# Move Robot to home position.
home_position = [-1.54 + math.pi, -1.83, -2.28, -0.59, 1.60, 0.023]
rtde_c.moveJ(home_position)

# Initialize the analogsticks controlling protocol for the joystick.
# Right and Left analogstick.
analogR = AnalogStickHandler(delta)
analogL = AnalogStickHandler(theta, 3)

analogs = [analogR, analogL]

# Function to move robot linear in tool-space(TCP) | joint-space(Joint angle).
move_robot = [(rtde_c.servoL, 'Cartesian'), (rtde_c.servoJ, 'Joint')]

funct = [move_robot_cartesian, move_robot_joint]
print(f'Robot control mode: {move_robot[0][1]}')

# vector: Pose of robot TCP(x, y, z, rx, ry, rz) or Angles in rad of
# the robot Joint(j0, j1, j2, j3, j4, j5).
vector = np.array(rtde_r.getActualTCPPose(), copy=False)
print(vector)

is_movable = False

while True:
    start = time.perf_counter()
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            button = controller.get_button(event.button)
            if button == 'R3':
                # Rotate the directions analogR axis controls.
                analogR.rotate_axis()
            if button == 'L3':
                # Rotate the directions analogL axis controls.
                analogL.rotate_axis()
            
            if button in ['SELECT', 'BACK']:
                # Switch mode: Cartesion-space to joint-space & vis-visa.
                funct.reverse()
                move_robot.reverse()
                mode = move_robot[0][1]
                vector = np.array(change_robot_mode(rtde_r, mode), copy=False)
    
            if button in ['START',]:
                # Start and Stop
                print('Stop Robot') if analogR.delta else print('Start Robot')
                analogR.delta, analogL.delta = [
                    0, 0] if analogR.delta else [delta, theta]

            if button in ['DPAD-UP', 'DPAD-DOWN']:
                hat = 1 if button in ['DPAD-UP'] else -1
                # rate += hat*10
                # dt = 1/rate
                analogR.delta += hat*rate 
                analogL.delta += hat*rate
                print(f'Left: {analogL.delta:.5f} Right: {analogR.delta:.5f}')
            
        if event.type == pygame.JOYHATMOTION:
            hat = event.value[1]
            if hat:
                # rate += hat*10
                # dt = 1/rate
                # print(f'Rate : {rate}Hz')
                ...
                analogR.delta += hat*rate 
                analogL.delta += hat*rate
            print(f'Left: {analogL.delta:.5f} Right: {analogR.delta:.5f}')


    is_movable = move_robot_command(
        joystick, controller, analogs, vector, funct[0])

    if is_movable:
        print(vector)
        M = move_robot[0][0]
        M(vector, velocity, acceleration, dt, lookahead_time, gain)

    end = time.perf_counter()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)
