
from numpy.lib.financial import rate
import yaml
from transforms import rpy2rv, rv2rpy
import os

# Extract robot configuration from the config file
with open('robot_config.yaml') as file:
    cfg = yaml.load(file, Loader=yaml.FullLoader)
    use_safety = cfg['use_safety']
    displacement_sq = cfg['displacement_sq']
    mount_surf = cfg['mount_surf']
    base_length = cfg['base_length']


class AnalogStickHandler:
    def __init__(self, delta=0, max_delta=0,offset=0, axis=['-X','+Y','+X','-Y']):
        self.delta = delta
        self.max_delta = max_delta
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


def limit_tcp(vector, idx, delta, cfg=[displacement_sq, mount_surf, base_length]):
    if not use_safety: return True
    # Maximum reach: (eq: d^2 = x^2 + y^2 + z^2) equation of a sphere with radius
    # equals to the maximum displacement of robob tcp.
    # Minimun reach: (eq: d^2 = x^2 + y^2) equation of a circle discribing the
    # imaginary cylinder around the robot base where the robot tcp should not reach
    # to prevent collision with itself.
    # h: correction for (eq. d^2 = x^2 + y^2 + (z+h)^2) when z < 0
    # Robot tcp center:(0,0,0) when z > 0 and (0,0,h) when z < 0

    vector[idx] += delta

    # Obtain current displacement^2
    h = 0 if vector[2] > 0 else -1*base_length
    current_min_dsq = vector[0]**2 + vector[1]**2
    current_max_dsq = vector[0]**2 + vector[1]**2 + (vector[2] + h)**2

    # Check minimum and maximum reach conditions
    min_dsq, max_dsq = displacement_sq
    min_reachable = current_min_dsq >= min_dsq
    max_reachable = current_max_dsq <= max_dsq

    #  Robot mount surface boundaries (rectangle)
    # [x_min, x_max, y_min, y_max, z_min] ->
    (x_min, x_max, y_min, y_max, z_min) = mount_surf

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


def limit_joint(vector, idx, delta, max_rotation=6.25):
    if not use_safety: return True
    # Absolute Rotation of robot joint not > max_rotation in radian.
    angle = vector[idx] + delta
    if abs(angle) <= max_rotation:
        return True

    return False


def move_robot_cartesian(vector, idx, delta):
    if idx > 2:
        # idx = 2 i.e. robot moving along z-axis
        # Conversion: [x, y, z, rx, ry, rz] -> [x, y, z, r, p, y].
        vector[3:] = rv2rpy(*vector[3:])
        vector[idx] += delta
        vector[3:] = rpy2rv(*vector[3:])

    elif limit_tcp(vector.copy(), idx, delta):
        vector[idx] += delta


def move_robot_joint(vector, idx, delta):
    if limit_joint(vector.copy(), idx, delta):
        vector[idx] += delta


def find_controller(controller_name, available_controllers):
    controller_name = controller_name.lower()
    pads = available_controllers.pads.keys()
    for pad in pads:
        if set(pad).issubset(set(controller_name)):
            return available_controllers.pads[pad]()

    raise Exception('Joystick controller not supported. Pls wait for update.')


def change_robot_mode(receiver, mode='Cartesian'):
    print(f'Robot control mode: {mode}')
    if mode == 'Cartesian':
        # [x, y, z, rx, ry, rz]
        return receiver.getActualTCPPose()
    elif mode == 'Joint':
        # [j0, j1, j2, j3, j4, j5]
        return receiver.getActualQ()


def print_logs(analogs):
    os.system('cls||clear')
    analogR, analogL = analogs
    dt = analogL.max_delta, analogR.max_delta
    print(f'L Analogstick: {analogL.delta:.5f} of {dt[0]:.5f}\n\n{analogL}')
    print(f'R Analogstick: {analogR.delta:.5f} of {dt[1]:.5f}\n\n{analogR}')


def increament_move_steps(analog, analogs, hat, rate):
    if not hat: return
    max_delta = analog.max_delta
    dt = round(analog.delta + hat*rate, 5)
    if (dt <= max_delta and dt > 0) or abs(dt) <= analog.delta:
        analog.delta += round(hat*rate, 5)

    print_logs(analogs)


def move_robot_command(joystick, controller, sticks, vector, F, noise=0.4):
    # Function F -> move_robot_cartesian | move_robot_joint.
    # Analogstick axis-value exceeds noise before any axis actions.
    delta0 = sticks[0].delta
    delta1 = sticks[1].delta

    # Default axis configuration and sign for referrencing.
    default = ['-X', '+Y', '+X', '-Y']
    sign = [-1, 1, 1, -1]

    get_axis = controller.get_axis
    get_button = controller.get_button

    # Change in vector element & controller trigger is_axis checker.
    is_axis = False
    is_movable = False

    if abs(v := joystick.get_axis(get_axis('RIGHT-X'))) >= noise:
        idx = default.index(sticks[0].axis[0])
        delta = sign[idx]*delta0
        idx = idx % 2 + sticks[0].offset
        F(vector, idx, delta) if v < 0 else F(vector, idx, -delta)
        is_movable += True

    if abs(v := joystick.get_axis(get_axis('RIGHT-Y'))) >= noise:
        idx = default.index(sticks[0].axis[1])
        delta = sign[idx]*delta0
        idx = idx % 2 + sticks[0].offset
        F(vector, idx, delta) if v < 0 else F(vector, idx, -delta)
        is_movable += True

    if abs(v := joystick.get_axis(get_axis('LEFT-X'))) >= noise:
        idx = default.index(sticks[1].axis[0])
        delta = sign[idx]*delta1
        idx = idx % 2 + sticks[1].offset
        F(vector, idx, delta) if v < 0 else F(vector, idx, -delta)
        is_movable += True

    if abs(v := joystick.get_axis(get_axis('LEFT-Y'))) >= noise:
        idx = default.index(sticks[1].axis[1])
        delta = sign[idx]*delta1
        idx = idx % 2 + sticks[1].offset
        F(vector, idx, delta) if v < 0 else F(vector, idx, -delta)
        is_movable += True

    if joystick.get_axis(get_axis('RT', 'R2')) > 0:
        idx = 2
        F(vector, idx, delta0)
        is_movable += True
        is_axis = True

    if joystick.get_axis(get_axis('LT', 'L2')) > 0:
        idx = 2
        F(vector, idx, -delta0)
        is_movable += True
        is_axis = True

    if not is_axis and joystick.get_button(get_button('RT', 'R2')) > 0:
        idx = 2
        F(vector, idx, delta0)
        is_movable += True

    if not is_axis and joystick.get_button(get_button('LT', 'L2')) > 0:
        idx = 2
        F(vector, idx, -delta0)
        is_movable += True

    if joystick.get_button(get_button('RB', 'R1')) > 0:
        idx = 5
        F(vector, idx, delta1)
        is_movable += True

    if joystick.get_button(get_button('LB', 'L1')) > 0:
        idx = 5
        F(vector, idx, -delta1)
        is_movable += True

    return is_movable
