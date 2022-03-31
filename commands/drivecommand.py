from commands2 import CommandBase
import constants

class DriveCommand(CommandBase):

    def __init__(self, robotContainer, controller):
        super().__init__()
        super().setName("DriveCommand")
        self.robotContainer = robotContainer
        self.controller = controller
        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        self.robotContainer.robotDrive.arcadeDrive(
            self.controller.getRawAxis(3) - self.controller.getRawAxis(2),
            self.controller.getRawAxis(0) * constants.kDrivetrainTurnSensitive)

        self.robotContainer.robotDrive.povDrive(
            self.controller.getPOV()
        )

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.robotDrive.tankDrive(0.0, 0.0)
