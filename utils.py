

from transforms import rpy2rv, rv2rpy


class AnalogStick:
    def __init__(self, joystick=None, delta=None, offset=0):
        self.joystick = joystick
        self.delta = delta
        self.offset = offset
        self.axis = ['-X', '+Y', '+X', '-Y']

    def rotate_axis(self, rotation='clockwise'):
        if rotation == 'clockwise':
            self.axis = [self.axis[-1], *self.axis[0:3]]
            # print(self.axis); quit()
        elif rotation == 'anticlockwise':
            self.axis = [*self.axis[1:4], self.axis[0]]

    def reset_axis(self):
        self.axis = ['-X', '+Y', '+X', '-Y']

    def __call__(self, **args):
        ...

    def __str__(self):
        return ('   {1}  \n{0}    {2}\n   {3}\n'.format(*self.axis))


class Pose:
    def __init__(self, pose=[]):
        self.pose = pose

    def __getitem__(self, key):
        return self.pose[key]
    
    
def limit_tcp(vector, idx, delta, minmax_displacemet=[], table=[]):
    vector[idx] += delta
    displacementXY = vector[0]**2 + vector[1]**2
    displacementXYZ = vector[0]**2 + vector[1]**2 + vector[2]**2 

    condition1 = displacementXY >= minmax_displacemet[0]
    condition2 = displacementXYZ <= minmax_displacemet[1]

    xbound = table[0] <= vector[0] >= table[2]
    ybound = table[1] <= vector[1] >= table[3]
    zbound = table[3] <= vector[2]

    if xbound and ybound and zbound:
        return False
    if not condition1 and condition2:
        return False

    return True

def limit_joint(vector, idx, delta, max_rotation = 6):
    angle = vector[idx] + delta
    if not max_rotation <= angle >= -max_rotation:
        return False

    return True


def move_robot_tcp(pose, idx, delta):
    if limit_tcp(pose[:], idx, delta):
        pose[idx] += delta


def move_robot_joint(pose, idx, delta):
    pose.pose[3:] = rv2rpy(*pose[3:])
    if limit_joint(pose[:], idx, delta):
        pose[idx] += delta    
    pose.pose[3:] = rpy2rv(*pose[3:])


def robot_command(sticks, pose, funct, signal=1):

    controller = []
    joystick = []

    delta0 = sticks[0].delta
    delta1 = sticks[1].delta

    get_axis = controller.get_axis
    get_button = controller.get_button
    
    if abs(val:=joystick.get_axis(get_axis('RIGHT-X'))) >= signal:
        x0 = sticks[0].axis.index('-X')%4 + sticks[0].offset
        x1 = (x0+2)%4 + sticks[0].offset
        funct(pose, x0, -delta0) if val < 0 else funct(pose, x1, delta0)

    if abs(val:=joystick.get_axis(get_axis('RIGHT-Y'))) >= signal:
        y0 = sticks[0].axis.index('-Y')%4 + sticks[0].offset
        y1 = (x0+2)%4 + sticks[0].offset
        funct(pose, y0, -delta0) if val < 0 else funct(pose, y1, delta0)

    if abs(val:=joystick.get_axis(get_axis('LEFT-X'))) >= signal:
        x0 = sticks[1].axis.index('-X')%4 + sticks[1].offset
        x1 = (x0+2)%4 + sticks[1].offset
        funct(pose, x0, -delta1) if val < 0 else funct(pose, x1, delta1)

    if abs(val:=joystick.get_axis(get_axis('LEFT-Y'))) >= signal:
        y0 = sticks[1].axis.index('-Y')%4 + sticks[1].offset
        y1 = (x0+2)%4 + sticks[1].offset
        funct(pose, y0, -delta1) if val < 0 else funct(pose, y1, delta1)

    
    if abs(joystick.get_axis(get_axis(['RT', 'R2']))) > 0:
        ...
    
    if abs(joystick.get_axis(get_axis(['LT', 'L2']))) > 0:
        ...
    
    if abs(joystick.get_button(get_button(['RB', 'R1']))) > 0:
        ...
    
    if abs(joystick.get_button(get_button(['LB', 'L1']))) > 0:
        ...

    if abs(joystick.get_button(get_button('RIGHT-Y'))):
        ...
    
    if abs(joystick.get_button(get_button('RIGHT-Y'))):
        ...

    
    
def is_movable():
    ...
