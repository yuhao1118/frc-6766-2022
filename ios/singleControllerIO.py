from wpilib import XboxController
from commands2 import Trigger

from ios.io2022 import IO2022


class SingleControllerIO(IO2022):
    def __init__(self, controllerPort):
        super().__init__()
        self.controller = XboxController(controllerPort)

    def getIntakerButton(self):
        return Trigger(self.controller.getLeftBumper)

    def getSimpleAimButton(self):
        return Trigger(self.controller.getRightBumper)

    def getGlobalAimButton(self):
        return Trigger()

    def getCargoIntakeButton(self):
        return Trigger(self.controller.getYButton)

    def getCargoOuttakeButton(self):
        return Trigger(self.controller.getAButton)

    def getShootButton(self):
        return Trigger(self.controller.getBButton)

    def getCompressorButton(self):
        return Trigger(self.controller.getXButton)

    def getClimbArmForwardButton(self):
        return Trigger(lambda: self.controller.getRightTriggerAxis() > 0.95)

    def getClimbArmBackwardButton(self):
        return Trigger(lambda: self.controller.getLeftTriggerAxis() > 0.95)

    def getPOVSupplier(self):
        return self.controller.getPOV

    def getDriveXSupplier(self):
        return lambda: -self.controller.getLeftY()

    def getDriveZSupplier(self):
        return self.controller.getRightX

    def getDebugButton(self):
        return Trigger(self.controller.getLeftStickButton)

    def getClimbElevatorPressSupplier(self):
        return self.controller.getStartButtonPressed
    