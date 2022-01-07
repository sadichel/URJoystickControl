
import yaml
import pygame
import controllers
import numpy as np
from transforms import rpy2rv, rv2rpy
from os import system


class AnalogStickHandler:
    def __init__(self, delta=0, offset=0, axis=['-X', '+Y', '+X', '-Y']):
        self.delta = delta
        self.offset = offset
        self.axis = axis
        self.default_axis = axis

    def rotate_axis(self, rotation='clockwise'):
        # Rotate axis config
        # clockwise: ['-X','+Y','+X','-Y'] -> ['-Y','-X','+Y','+X']
        if rotation == 'clockwise':
            self.axis = [self.axis[-1], *self.axis[0:3]]
        if rotation == 'anticlockwise':
            self.axis = [*self.axis[1:4], self.axis[0]]

    def reset_axis(self):
        self.axis = self.default_axis

    def __str__(self):
        return ('   {1}  \n{0}    {2}\n   {3}\n'.format(*self.axis))


class JoystickHandler:
    # Extends Pygame.joystick.Joystick for applicability
    def __init__(self, joystick):
        self.joystick = joystick

    def get_axis(self, axis_number: int) -> float:
        try:
            return self.joystick.get_axis(axis_number)
        except:
            return 0

    def get_button(self, button_number: int) -> bool:
        try:
            return self.joystick.get_button(button_number)
        except:
            return 0


def init_joystick(id=0):
    # Initialize Pygame Joystick and select controller
    pygame.init()
    id = 0
    joystick = pygame.joystick.Joystick(id)
    joystick.init()
    print(joystick.get_name())
    if joystick.get_numaxes() in [4, 6]:
        pads = joystick.get_name()
        controller_map = find_controller(pads, controllers.CONTROLLERS)
    return JoystickHandler(joystick), controller_map


def limit_tcp(vector, idx, delta, cfg):
    if not cfg['use_safety']:
        return True
    vector[idx] += delta
    # Maximum sphere: [d^2 = x^2 + y^2 + (z+h)^2]
    # Minimun cylinder: [d^2 = x^2 + y^2]
    # Center (0,0,0) when z > 0 and (0,0,h) when z < 0
    h = 0 if vector[2] > 0 else -1*cfg['base_length']
    current_min_dsq = vector[0]**2 + vector[1]**2
    current_max_dsq = vector[0]**2 + vector[1]**2 + (vector[2] + h)**2

    # Check minimum and maximum reach conditions
    min_dsq, max_dsq = cfg['displacement_sq']
    min_reachable = current_min_dsq >= min_dsq
    max_reachable = current_max_dsq <= max_dsq

    #  Robot mount surface boundaries (rectangle)
    # [x_min, x_max, y_min, y_max, z_min] ->
    (x_min, x_max, y_min, y_max, z_min) = cfg['mount_surf']

    # Check boundaries of the mount surface: collision
    x_bound = vector[0] >= x_min and vector[0] <= x_max
    y_bound = vector[1] >= y_min and vector[1] <= y_max
    z_bound = vector[2] >= z_min

    # idx = 2 i.e robot moving along the z-axis
    if idx == 2 and not z_bound:
        if x_bound and y_bound:
            return False
    # Reachable robot tcp orientation
    if not (min_reachable and max_reachable):
        return False
    return True


def limit_joint(vector, idx, delta, cfg, max_rotation=6.25):
    # Absolute Rotation of robot joint not > max_rotation in radian
    if not cfg['use_safety']:
        return True
    angle = vector[idx] + delta
    if abs(angle) <= max_rotation:
        return True
    return False


def moveit_cartesian_space(vector, idx, delta, cfg):
    if idx > 2:
        # idx = 2 i.e. robot moving along z-axis
        # [x, y, z, rx, ry, rz] -> [x, y, z, r, p, y].
        vector[3:] = rv2rpy(*vector[3:])
        vector[idx] += delta
        vector[3:] = rpy2rv(*vector[3:])
    elif limit_tcp(vector.copy(), idx, delta, cfg):
        vector[idx] += delta


def moveit_joint_space(vector, idx, delta, cfg):
    if limit_joint(vector.copy(), idx, delta, cfg):
        vector[idx] += delta


def find_controller(controller_name, available_controllers):
    controller_name = controller_name.lower()
    pads = available_controllers.pads.keys()
    for pad in pads:
        if set(pad).issubset(set(controller_name)):
            return available_controllers.pads[pad]()
    raise Exception('Joystick controller not supported. Pls wait for update.')


def change_moveit_mode(vector, rtde, mode='Cartesian'):
    # [x, y, z, rx, ry, rz]
    # [j0, j1, j2, j3, j4, j5]
    if mode == 'Cartesian':
        vector[:] = np.array(rtde.getActualTCPPose())
    elif mode == 'Joint':
        vector[:] = np.array(rtde.getActualQ())


def print_logs(params):
    system('cls||clear')
    state = 'running' if params.state else 'paused'
    print(f'Mode: {params.modes[0]:<9}   state: {state} \n')
    analogs = [params.analogR, params.analogL]
    max_delta = [params.max_delta, params.max_theta]
    n = ['R', 'L']
    for i, analog in enumerate(analogs):
        dt = analog.delta, max_delta[i]
        line = f'Analog: {n[i]}   steps: [{dt[0]:.5f}][{dt[1]:.5f}]'
        print(f'{line}\n\n{analogs[i]}')


def steps(analog, max_delta, hat, rate):
    if not hat:
        return
    dt = round(analog.delta + hat*rate, 5)
    if (dt <= max_delta and dt > 0) or abs(dt) <= analog.delta:
        analog.delta += round(hat*rate, 5)


class ControlParams:
    def __init__(self, delta, theta, navigator, config_file=None):
        self.start = False
        self.state = True
        # [delta, max_delta], [theta, max_theta] = delta, theta
        # Increase steps: delta += rate, theta += rate
        self.rate = 0.00005
        self.modes = ['Cartesian', 'Joint']
        self.max_delta = delta*4
        self.max_theta = theta*3.142
        # TCP p(x, y, z, rx, ry, rz) & Joint p(j0, j1, j2, j3, j4, j5)
        # analogR delta controls p[0:3]:[x, y, z] | [j0, j1, j2]
        # analogL theta controls p[3:6]:[r, p, y] | [j3, j4, j5]
        self.analogR = AnalogStickHandler(delta)
        self.analogL = AnalogStickHandler(theta, 3,
                                          axis=['+Y', '+X', '-Y', '-X'])

        self.funct = [moveit_cartesian_space, moveit_joint_space]
        self.navigator = navigator
        # Extract robot configuration from the config file
        with open(config_file or 'robot_config.yaml') as file:
            self.cfg = yaml.load(file, Loader=yaml.FullLoader)


def joystick_axis_actions(joystick, controller, params, vector, noise=0.4):
    if not params.start or not params.state:
        return False
    # Function F -> move_robot_cartesian | move_robot_joint.
    # Analogstick axis-value exceeds noise before any axis actions.
    F = params.funct[0]
    cfg = params.cfg
    analogs = params.analogR, params.analogL
    delta0 = analogs[0].delta
    theta0 = analogs[1].delta

    # Default axis configuration and sign for referrencing.
    default = ['-X', '+Y', '+X', '-Y']
    sign = [-1, 1, 1, -1]

    get_axis = controller.get_axis
    get_button = controller.get_button

    # Change in vector element & controller trigger is_axis checker.
    is_axis = False
    event_records = False

    if abs(v := joystick.get_axis(get_axis('RIGHT-X'))) >= noise:
        idx = default.index(analogs[0].axis[0])
        delta = sign[idx]*delta0
        idx = idx % 2 + analogs[0].offset
        F(vector, idx, delta, cfg) if v < 0 else F(vector, idx, -delta, cfg)
        event_records += True

    if abs(v := joystick.get_axis(get_axis('RIGHT-Y'))) >= noise:
        idx = default.index(analogs[0].axis[1])
        delta = sign[idx]*delta0
        idx = idx % 2 + analogs[0].offset
        F(vector, idx, delta, cfg) if v < 0 else F(vector, idx, -delta, cfg)
        event_records += True

    if abs(v := joystick.get_axis(get_axis('LEFT-X'))) >= noise:
        idx = default.index(analogs[1].axis[0])
        delta = sign[idx]*theta0
        idx = idx % 2 + analogs[1].offset
        F(vector, idx, delta, cfg) if v < 0 else F(vector, idx, -delta, cfg)
        event_records += True

    if abs(v := joystick.get_axis(get_axis('LEFT-Y'))) >= noise:
        idx = default.index(analogs[1].axis[1])
        delta = sign[idx]*theta0
        idx = idx % 2 + analogs[1].offset
        F(vector, idx, delta, cfg) if v < 0 else F(vector, idx, -delta, cfg)
        event_records += True

    if joystick.get_axis(get_axis('RT', 'R2')) > 0:
        idx = 2
        F(vector, idx, delta0, cfg)
        event_records += True
        is_axis = True

    if joystick.get_axis(get_axis('LT', 'L2')) > 0:
        idx = 2
        F(vector, idx, -delta0, cfg)
        event_records += True
        is_axis = True

    if not is_axis and joystick.get_button(get_button('RT', 'R2')) > 0:
        idx = 2
        F(vector, idx, delta0, cfg)
        event_records += True

    if not is_axis and joystick.get_button(get_button('LT', 'L2')) > 0:
        idx = 2
        F(vector, idx, -delta0, cfg)
        event_records += True

    if joystick.get_button(get_button('RB', 'R1')) > 0:
        idx = 5
        F(vector, idx, theta0, cfg)
        event_records += True

    if joystick.get_button(get_button('LB', 'L1')) > 0:
        idx = 5
        F(vector, idx, -theta0, cfg)
        event_records += True

    return event_records


def joystick_button_actions(rtde, controller, params, vector):
    event_records = False
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:  # start guide
            if not params.start:
                params.start = True
            event_records += True

        if event.type == pygame.JOYHATMOTION:
            if hat := event.value[0]:
                steps(params.analogL, params.max_theta, hat, params.rate)
            if hat := event.value[1]:
                steps(params.analogR, params.max_delta, hat, params.rate)
            event_records += True
            print_logs(params)

        if event.type == pygame.JOYBUTTONDOWN:
            if not params.start:  # start guide
                params.start = True

            button = controller.get_button(event.button)
            if button == 'R3':
                # Rotate the directions analogR axis controls.
                params.analogR.rotate_axis()
                print_logs(params)
                
            if button == 'L3':
                # Rotate the directions analogL axis controls.
                params.analogL.rotate_axis()
                print_logs(params)

            if button in ['SELECT', 'BACK']:
                # Switch mode: Cartesion-space to joint-space & vis-visa.
                params.funct.reverse()
                params.navigator.reverse()
                params.modes.reverse()
                change_moveit_mode(vector, rtde[0], params.modes[0])
                print_logs(params)

            if button in ['START']:
                # Start and Stop
                params.state = True if not params.state else False
                print_logs(params)

            if button in ['DPAD-UP', 'DPAD-DOWN']:
                hat = 1 if button in ['DPAD-UP'] else -1
                steps(params.analogR, params.max_delta, hat, params.rate)
                print_logs(params)

            if button in ['DPAD-LEFT', 'DPAD-RIGHT']:
                hat = 1 if button in ['DPAD-RIGHT'] else -1
                steps(params.analogL, params.max_theta, hat, params.rate)
                print_logs(params)

            if button in ['X']:
                # rtde[1].disconnect() if rtde[1].isConnected(
                # ) else rtde[1].reconnect()
                ...

            event_records += True

    return event_records
