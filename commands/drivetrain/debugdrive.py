import imp
from commands2 import CommandBase

from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.tunablenumber import TunableNumber


class DebugDrive(CommandBase):
    def __init__(self, robotContainer):
        super().__init__()
        super().setName("TuningDrive")

        self.robotContainer = robotContainer
        self.linearX = TunableNumber("Drivetrain/tuneWheelSpeed")
        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        speeds = WheelSpeedsPercentage.fromArcade(float(self.linearX), 0.0)
        self.robotContainer.robotDrive.tankDriveVelocity(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.robotDrive.tankDrive(0.0, 0.0)
