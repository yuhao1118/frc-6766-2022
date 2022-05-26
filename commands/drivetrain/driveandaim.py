from commands2 import CommandBase
from wpilib.geometry import Pose2d
from wpimath.controller import PIDController

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.interpolatedict import InterpolateDict
from lib.utils.tunablenumber import TunableNumber
from lib.utils.maths import clamp, axisProfile

import math


class DriveAimCommand(CommandBase):
    """
    自瞄并自动测距指令

    输入:
        robotContainer: RobotContainer实例
        goalAngle: 目标角度, 默认0度
        controller=None: 手柄控制器, 默认None
    """

    def __init__(self, robotContainer, controller=None):
        super().__init__()
        super().setName("DriveAimCommand")
        self.robotContainer = robotContainer
        self.kP = TunableNumber("Aim/kP", 0.0055)
        self.kI = TunableNumber("Aim/kI", 0.002)
        self.kD = TunableNumber("Aim/kD", 0.1)

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )
        self.turnPidController.setTolerance(positionTolerance=0.035)
        self.turnPidController.enableContinuousInput(-math.pi, math.pi)
        self.goalAngle = 0.0
        self.controller = controller

        self.correctedPose = Pose2d()

        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        speeds = WheelSpeedsPercentage(0, 0)

        if self.kP.hasChanged():
            self.turnPidController.setP(self.kP.get())
        if self.kI.hasChanged():
            self.turnPidController.setI(self.kI.get())
        if self.kD.hasChanged():
            self.turnPidController.setD(self.kD.get())

        if self.controller is not None:
            linearX = axisProfile(-self.controller.getRawAxis(1))
            angularZ = axisProfile(self.controller.getRawAxis(4))

            arcadeSpeeds = WheelSpeedsPercentage.fromArcade(
                linearX, angularZ * constants.kDrivetrainTurnSensitive)
            curvatureSpeeds = WheelSpeedsPercentage.fromCurvature(
                linearX, angularZ)

            hybridScale = clamp(
                abs(linearX) / constants.kCurvatureThreshold, 0, 1)
            speeds = WheelSpeedsPercentage(
                curvatureSpeeds.left * hybridScale
                + arcadeSpeeds.left * (1 - hybridScale),
                curvatureSpeeds.right * hybridScale
                + arcadeSpeeds.right * (1 - hybridScale))

        # Adjust drivetrain to face the hub
        xOffset = self.robotContainer.visionControl.getXOffset().radians()
        distance = self.robotContainer.visionControl.getDistanceMeters()

        turnSpeed = self.turnPidController.calculate(xOffset, self.goalAngle)
        if self.turnPidController.atSetpoint():
            turnSpeed = 0.0

        speeds = speeds + WheelSpeedsPercentage.fromArcade(0.0, turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
