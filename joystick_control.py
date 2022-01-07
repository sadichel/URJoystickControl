import math
import time
import numpy as np
from utils import print_logs
from utils import ControlParams
from utils import init_joystick
from utils import joystick_axis_actions
from utils import joystick_button_actions
from argparse import ArgumentParser
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

np.set_printoptions(precision=4, floatmode='fixed', suppress=True)

parser = ArgumentParser()
parser.add_argument('--robot_ip', default='localhost',
                    help='Universal robot IP address')
parser.add_argument('--boundary_config', default='robot_boundary_config.yaml',
                    help='Satety config setup')


def main():
    args = parser.parse_args()
    robot_ip = args.robot_ip
    cfg_file = args.boundary_config
    rtde_r = RTDEReceiveInterface(robot_ip)
    rtde_c = RTDEControlInterface(robot_ip)
    rtde = [rtde_r, rtde_c]
    print('Connection Established with Univeral Robot')
    # Initialize Pygame Joystick and select controller
    joystick, controller_map = init_joystick(id=0)

    # URScript parameters: dt = 1/500Hz transmission rate
    acceleration = 0.5
    velocity = 0.25
    dt = 1.0/500
    lookahead_time = 0.1
    gain = 300

    # Move steps parameters
    delta = 0.00025
    theta = delta * math.pi
    
    # Move UR Robot to home position.
    home_position = [-1.54 + math.pi, -1.83, -2.28, -0.59, 1.60, 0.023]
    rtde_c.moveJ(home_position)

    # Linear servo in tool-space(TCP) | joint-space(Joint angle).
    navigator = [rtde_c.servoL, rtde_c.servoJ]

    # vector:TCP p(x, y, z, rx, ry, rz) | Joint p(j0, j1, j2, j3, j4, j5).
    vector = np.array(rtde_r.getActualTCPPose(), copy=False)
    params = ControlParams(delta, theta, navigator, cfg_file)

    print_logs(params)
    print(vector)

    while True:
        start = time.perf_counter()
        joystick_button_actions(rtde, controller_map, params, vector)

        movable = joystick_axis_actions(
            joystick, controller_map, params, vector)

        if movable:
            print(vector)
            M = params.navigator[0]
            M(vector, velocity, acceleration, dt, lookahead_time, gain)

        end = time.perf_counter()
        duration = end - start
        if duration < dt:
            time.sleep(dt - duration)


if __name__ == '__main__':
    main()
