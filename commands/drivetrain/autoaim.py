import math

from commands2 import CommandBase
from wpilib import Timer
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.tunablenumber import TunableNumber
from lib.utils.maths import clamp, axisProfile


def getTargetRotation(position):
    vehicleToHub = constants.kHubCenter - position
    return Rotation2d(vehicleToHub.X(), vehicleToHub.Y())


class AutoAim(CommandBase):
    """
    自瞄指令

    输入:
        robotContainer: RobotContainer实例
        controller=None: 手柄控制器, 默认None
    """

    def __init__(self, robotContainer, controller=None):
        super().__init__()
        super().setName("AutoAim")
        self.robotContainer = robotContainer
        self.kP = TunableNumber("AutoAim/kP", 0.0029)
        self.kI = TunableNumber("AutoAim/kI", 0.0)
        self.kD = TunableNumber("AutoAim/kD", 0.0005)
        self.integralMaxError = TunableNumber("AutoAim/IntegralMaxError", 0.0)
        self.minVelocity = TunableNumber("AutoAim/MinVelocity", 0.0)
        self.tolerenceDegrees = TunableNumber("AutoAim/tolerenceDegrees", 3.0)
        self.tolerenceTime = TunableNumber("AutoAim/toleranceTime", 0.3)

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )
        self.turnPidController.setTolerance(positionTolerance=self.tolerenceDegrees.getDefault())
        self.turnPidController.enableContinuousInput(-180.0, 180.0)

        self.tolerenceTimer = Timer()
        self.controller = controller

        self.addRequirements(self.robotContainer.robotDrive)

    def initialize(self):
        self.robotContainer.visionControl.setPipeline(1)
        self.turnPidController.reset()
        self.tolerenceTimer.reset()
        self.tolerenceTimer.start()

    def execute(self):
        speeds = WheelSpeedsPercentage(0, 0)

        if self.kP.hasChanged():
            self.turnPidController.setP(float(self.kP))
        if self.kI.hasChanged():
            self.turnPidController.setI(float(self.kI))
        if self.kD.hasChanged():
            self.turnPidController.setD(float(self.kD))
        if self.tolerenceDegrees.hasChanged():
            self.turnPidController.setTolerance(positionTolerance=float(self.tolerenceDegrees))

        self.turnPidController.setSetpoint(
            getTargetRotation(self.robotContainer.robotState.getLatestPose().translation()).degrees()
        )

        if not self.turnPidController.atSetpoint():
            self.tolerenceTimer.reset()

        if abs(self.turnPidController.getPositionError()) >= float(self.integralMaxError):
            self.turnPidController.setI(0.0)
        else:
            self.turnPidController.setI(float(self.kI))

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

        # Adjust drivetrain to aim at the hub
        turnSpeed = self.turnPidController.calculate(self.robotContainer.robotState.getLatestRotation().degrees())
        if abs(turnSpeed) < float(self.minVelocity):
            turnSpeed = math.copysign(float(self.minVelocity), turnSpeed)

        speeds = speeds + WheelSpeedsPercentage.fromArcade(0.0, -turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return self.tolerenceTimer.hasElapsed(self.tolerenceTime.get())

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.tolerenceTimer.stop()
