from commands2 import CommandBase
from wpimath.controller import PIDController

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage

class AimCommand(CommandBase):
    """
    自瞄并自动测距指令
    
    输入:
        robotContainer: RobotContainer实例
        goalAngle: 目标角度, 默认0度

    """
    def __init__(self, robotContainer, goalAngle=0):
        super().__init__()
        self.robotContainer = robotContainer

        self.turnPidController = PIDController(
            constants.kPVisionTurn,
            constants.kIVisionTurn,
            constants.kDVisionTurn
        )
        self.turnPidController.setTolerance(positionTolerance=2.0)

        self.goalAngle = goalAngle

        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        turnSpeed = 0
        
        angle = self.robotContainer.visionControl.getRotation2d().degrees()
        turnSpeed = -self.turnPidController.calculate(angle, self.goalAngle)
        
        if self.turnPidController.atSetpoint():
            turnSpeed = 0.0

        speeds = WheelSpeedsPercentage.fromArcade(0.0, turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
