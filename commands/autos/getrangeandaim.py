from commands2 import CommandBase
from wpimath.controller import PIDController

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage

class GetRangeAndAimCommand(CommandBase):
    """
    自瞄并自动测距指令
    
    输入:
        robotContainer: RobotContainer实例
        goalRange: 目标距离, 默认0.35m
        goalAngle: 目标角度, 默认0度
        shouldAutoRanging: 是否需要自动测距, 默认False, 意思是只自瞄, 不自动测距
        
        注: 此处的自动调整距离是指通过移动底盘至目标距离开始射球
        并非<射球指令>中，通过调节电机转速, 使得在当前位置完成射球. 
    """
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
        self.turnPidController.setTolerance(positionTolerance=2.0)

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
        if self.turnPidController.atSetpoint():
            turnSpeed = 0.0

        speeds = WheelSpeedsPercentage.fromArcade(forwardSpeed, turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
