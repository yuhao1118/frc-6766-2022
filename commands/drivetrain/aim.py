from commands2 import CommandBase
from wpimath.controller import PIDController
from wpilib import SmartDashboard

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.interpolatedict import InterpolateDict
from lib.utils.tunablenumber import TunableNumber

HoodDistanceAngle = InterpolateDict({
    0.0: 0.0,
    4.0: 20.0
})

class AimCommand(CommandBase):
    """
    自瞄并自动测距指令
    
    输入:
        robotContainer: RobotContainer实例
        goalAngle: 目标角度, 默认0度
        linearXSuppiler: 线速度输入, 默认为0
    """
    def __init__(self, robotContainer, goalAngle=0, linearXSuppiler=lambda: 0.0):
        super().__init__()
        self.robotContainer = robotContainer
        self.turnPidController = PIDController(
            0.0055,
            0.002,
            0.1
        )
        self.turnPidController.setTolerance(positionTolerance=2.0)
        self.goalAngle = goalAngle
        self.linearXSuppiler = linearXSuppiler
        
        self.hoodAngle = TunableNumber("Hood/Angle", 12.0)

        self.addRequirements(self.robotContainer.robotDrive, self.robotContainer.hoodDrive)

    def execute(self):
        # Adjust drivetrain to face the hub
        turnAngle = self.robotContainer.visionControl.getRotation2d().degrees()
        turnSpeed = -self.turnPidController.calculate(turnAngle, self.goalAngle)
        if self.turnPidController.atSetpoint():
            turnSpeed = 0.0
        speeds = WheelSpeedsPercentage.fromArcade(self.linearXSuppiler(), turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

        # Adjust hood for shooting
        distance = self.robotContainer.visionControl.getDistance()
        # if distance is not None:
        #     angle = HoodDistanceAngle.getInterpolated(distance)
        #     self.robotContainer.hoodDrive.setAngle(angle)
        # else:
        self.robotContainer.hoodDrive.setAngle(float(self.hoodAngle))

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.robotContainer.hoodDrive.set(0.0)
