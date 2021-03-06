
"""
Standard gamepad controller mappings.

"""


class Controller:
    def get_axis(self, *axes):
        for axis in axes:
            if self.axis.get(axis) != None:
                return self.axis.get(axis)
        return None

    def get_button(self, *buttons):
        for button in buttons:
            if self.button.get(button) != None:
                return self.button.get(button)
        return None


class LogitechF710(Controller):
    def __init__(self):
        self.axis = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'LT',
            3: 'RIGHT-X',
            4: 'RIGHT-Y',
            5: 'RT',
            'LEFT-X': 0,
            'LEFT-Y': 1,
            'LT': 2,
            'RIGHT-X': 3,
            'RIGHT-Y': 4,
            'RT': 5
        }
        self.button = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            6: 'BACK',
            7: 'START',
            8: 'LOGITECH',
            9: 'L3',
            10: 'R3',
            'A': 0,
            'B': 1,
            'X': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,
            'BACK': 6,
            'START': 7,
            'LOGITECH': 8,
            'L3': 9,
            'R3': 10
        }


class LogitechRumblePad(Controller):
    def __init__(self):
        self.axis = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'RIGHT-X',
            3: 'RIGHT-Y',
            'LEFT-X': 0,
            'LEFT-Y': 1,
            'RIGHT-X': 2,
            'RIGHT-Y': 3,
        }
        self.button = {
            0: 'X',
            1: 'A',
            2: 'B',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            6: 'LT',
            7: 'RT',
            8: 'BACK',
            9: 'START',
            10: 'L3',
            11: 'R3',
            'A': 0,
            'B': 1,
            'X': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,
            'LT': 6,
            'RT': 7,
            'BACK': 8,
            'START': 9,
            'L3': 10,
            'R3': 11
        }


class PS3(Controller):
    def __init__(self):
        self.axis = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'L2',
            3: 'RIGHT-X',
            4: 'RIGHT-Y',
            5: 'R2',
            'LEFT-X': 0,
            'LEFT-Y': 1,
            'L2': 2,
            'RIGHT-X': 3,
            'RIGHT-Y': 4,
            'R3': 5
        }
        self.button = {
            0:  'CROSS',
            1:  'CIRCLE',
            2:  'TRIANGLE',
            3:  'SQUARE',
            4:  'L1',
            5:  'R1',
            6:  'L2',
            7:  'R2',
            8:  'SELECT',
            9:  'START',
            10: 'PS',
            11: 'L3',
            12: 'R3',
            13: 'DPAD-UP',
            14: 'DPAD-DOWN',
            15: 'DPAD-LEFT',
            16: 'DPAD-RIGHT',
            'CROSS': 0,
            'CIRCLE': 1,
            'TRIANGLE': 2,
            'SQUARE': 3,
            'L1': 4,
            'R1': 5,
            'L2': 6,
            'R2': 7,
            'SELECT': 8,
            'START': 9,
            'PS': 10,
            'L3': 11,
            'R3': 12,
            'DPAD-UP': 13,
            'DPAD-DOWN': 14,
            'DPAD-LEFT': 15,
            'DPAD-RIGHT': 16
        }


class PS4(Controller):
    def __init__(self):
        self.axis = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'L2',
            3: 'RIGHT-X',
            4: 'RIGHT-Y',
            5: 'R2',
            6: 'DPAD-X',
            7: 'DPAD-Y',
            'LEFT-X': 0,
            'LEFT-Y': 1,
            'L2': 2,
            'RIGHT-X': 3,
            'RIGHT-Y': 4,
            'R2': 5,
            'DPAD-X': 6,
            'DPAD-Y': 7,
        }
        self.button = {
            0:  'CROSS',
            1:  'CIRCLE',
            2:  'TRIANGLE',
            3:  'SQUARE',
            4:  'L1',
            5:  'R1',
            6:  'L2',
            7:  'R2',
            8:  'SHARE',
            9:  'OPTIONS',
            10: 'PS',
            11: 'L3',
            12: 'R3',
            'CROSS': 0,
            'CIRCLE': 1,
            'TRIANGLE': 2,
            'SQUARE': 3,
            'L1': 4,
            'R1': 5,
            'L2': 6,
            'R2': 7,
            'SHARE': 8,
            'OPTIONS': 9,
            'PS': 10,
            'L3': 11,
            'R3': 12
        }


class XBox360(Controller):
    def __init__(self):
        self.axis = {
            0: 'LEFT-X',
            1: 'LEFT-Y',
            2: 'LT',
            3: 'RIGHT-X',
            4: 'RIGHT-Y',
            5: 'RT',
            'LEFT-X': 0,
            'LEFT-Y': 1,
            'LT': 2,
            'RIGHT-X': 3,
            'RIGHT-Y': 4,
            'RT': 5
        }
        self.button = {
            0:  'A',
            1:  'B',
            2:  'X',
            3:  'Y',
            4:  'LB',
            5:  'RB',
            6:  'BACK',
            7:  'START',
            8:  'XBOX',
            9:  'L3',
            10: 'R3',
            'A': 0,
            'B': 1,
            'X': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,
            'BACK': 6,
            'START': 7,
            'XBOX': 8,
            'L3': 9,
            'R3': 10
        }


class CONTROLLERS:
    pads = {
        'ps3': PS3,
        'ps4': PS4,
        'xbox': XBox360,
        'f310': LogitechF710,
        'f710': LogitechF710,
        'rumblepad1': LogitechRumblePad,
        'rumblepad2': LogitechRumblePad
    }
