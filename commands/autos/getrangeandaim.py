from commands2 import CommandBase
from wpimath.controller import PIDController

import constants


class GetRangeAndAimCommand(CommandBase):
    def __init__(self, robotContainer, goalRange=0.35, goalAngle=0, shouldAutoRanging=False):
        super().__init__()
        self.robotContainer = robotContainer
        self.shouldAutoRanging = shouldAutoRanging
        self.forwardPidController = PIDController(
            constants.kPVisionForward,
            constants.kIVisionForward,
            constants.kDVisionForward)

        self.turnPidController = PIDController(
            constants.kPVisionTurn,
            constants.kIVisionTurn,
            constants.kDVisionTurn
        )

        self.goalRange = goalRange
        self.goalAngle = goalAngle

        self.addRequirements(self.robotContainer.robotDrive, self.robotContainer.visionControl)

    def execute(self):
        forwardSpeed = 0
        turnSpeed = 0

        if self.shouldAutoRanging:
            range = self.robotContainer.visionControl.getDistance()
            if 0 < range - self.goalRange < 5:
                forwardSpeed = -self.forwardPidController.calculate(
                    range, self.goalRange)
        
        angle = self.robotContainer.visionControl.getRotation2d().degrees()
        turnSpeed = -self.turnPidController.calculate(angle, self.goalAngle)

        self.robotContainer.robotDrive.arcadeDrive(
            forwardSpeed, turnSpeed, squareInputs=False)

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.arcadeDrive(0, 0)
