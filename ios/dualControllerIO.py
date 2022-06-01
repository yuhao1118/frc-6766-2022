from wpilib import XboxController
from commands2 import Trigger

from ios.io2022 import IO2022


class DualControllerIO(IO2022):
    def __init__(self, driverControllerPort, operatorControllerPort):
        super().__init__()
        self.driverController = XboxController(driverControllerPort)
        self.operatorController = XboxController(operatorControllerPort)

    def getSimpleAimButton(self):
        return Trigger(self.driverController.getRightBumper)

    def getGlobalAimButton(self):
        return Trigger(self.driverController.getLeftBumper)

    def getIntakerButton(self):
        return Trigger(self.operatorController.getLeftBumper)

    def getCargoIntakeButton(self):
        return Trigger(self.operatorController.getYButton)

    def getCargoOuttakeButton(self):
        return Trigger(self.operatorController.getAButton)

    def getShootButton(self):
        return Trigger(self.operatorController.getBButton)

    def getCompressorButton(self):
        return Trigger(self.operatorController.getXButton)

    def getClimbArmForwardButton(self):
        return Trigger(self.driverController.getBButton)

    def getClimbArmBackwardButton(self):
        return Trigger(self.driverController.getXButton)

    def getClimbElevatorUpButton(self):
        return Trigger(self.driverController.getYButton)

    def getClimbElevatorDownButton(self):
        return Trigger(self.driverController.getAButton)

    def getPOVSupplier(self):
        return self.driverController.getPOV

    def getDriveXSupplier(self):
        return lambda: self.driverController.getLeftTriggerAxis() - self.driverController.getRightTriggerAxis()

    def getDriveZSupplier(self):
        return self.driverController.getLeftX

    def getDebugButton(self):
        return Trigger(self.driverController.getStartButton)
