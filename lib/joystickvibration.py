from commands2 import RunCommand
from wpilib import Joystick
from lib.enums.rumble import JoystickRumble


def JoyStickVibration(controller, rumbleType, strength=0.5):
    if rumbleType == JoystickRumble.kLeftRumble:
        controller.setRumble(Joystick.RumbleType.kLeftRumble, strength)
    elif rumbleType == JoystickRumble.kRightRumble:
        controller.setRumble(Joystick.RumbleType.kRightRumble, strength)
    elif rumbleType == JoystickRumble.kBothRumble:
        controller.setRumble(Joystick.RumbleType.kLeftRumble, strength)
        controller.setRumble(Joystick.RumbleType.kRightRumble, strength)


def JoyStickVibrationCommand(controller, rumbleType, strength=0.5, duration=0.5):
    return RunCommand(lambda: JoyStickVibration(controller, rumbleType, strength)).withTimeout(duration)
