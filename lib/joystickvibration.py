from commands2 import RunCommand
from wpilib import Joystick
from lib.enums.rumble import JoystickRumble


def JoyStickVibrationCommand(controller, rumbleType, strength=0.5, duration=0.5):
    def rumble():
        if rumbleType == JoystickRumble.kLeftRumble:
            controller.setRumble(Joystick.RumbleType.kLeftRumble, strength)
        elif rumbleType == JoystickRumble.kRightRumble:
            controller.setRumble(Joystick.RumbleType.kRightRumble, strength)
        elif rumbleType == JoystickRumble.kBothRumble:
            controller.setRumble(Joystick.RumbleType.kLeftRumble, strength)
            controller.setRumble(Joystick.RumbleType.kRightRumble, strength)

    return RunCommand(rumble).withTimeout(duration)