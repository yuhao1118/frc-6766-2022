import math

from commands2 import CommandBase
from wpilib import Timer
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.axisProfile import AxisProfile
from lib.utils.tunablenumber import TunableNumber


def getTargetRotation(position):
    vehicleToHub = constants.kHubCenter - position
    return Rotation2d(vehicleToHub.X(), vehicleToHub.Y())


class AutoAim(CommandBase):
    """
    自瞄指令

    输入:
        robotContainer: RobotContainer实例
    """

    def __init__(self, robotContainer, io=None, shouldAutoTerminate=True):
        super().__init__()
        super().setName("AutoAim")
        self.robotContainer = robotContainer

        self.linearXProfiler = None
        self.linearXSupplier = None

        if io is not None:
            self.linearXProfiler = AxisProfile(0.04)
            self.linearXSupplier = io.getDriveXSupplier()

        self.kP = TunableNumber("AutoAim/kP", 0.0029)
        self.kI = TunableNumber("AutoAim/kI", 0.0)
        self.kD = TunableNumber("AutoAim/kD", 0.0005)
        self.integralMaxError = TunableNumber("AutoAim/IntegralMaxError", 0.0)
        self.minVelocity = TunableNumber("AutoAim/MinVelocity", 0.0)
        self.tolerenceDegrees = TunableNumber("AutoAim/tolerenceDegrees", 3.0)
        self.tolerenceTime = TunableNumber("AutoAim/toleranceTime", 0.3)
        self.shouldAutoTerminate = shouldAutoTerminate

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )
        self.turnPidController.setTolerance(positionTolerance=self.tolerenceDegrees.getDefault())
        self.turnPidController.enableContinuousInput(-180.0, 180.0)

        self.tolerenceTimer = Timer()

        self.addRequirements(self.robotContainer.robotDrive)

    def initialize(self):
        self.turnPidController.reset()
        self.tolerenceTimer.reset()
        self.tolerenceTimer.start()

    def execute(self):
        speeds = WheelSpeedsPercentage(0.0, 0.0)

        if self.kP.hasChanged():
            self.turnPidController.setP(float(self.kP))
        if self.kI.hasChanged():
            self.turnPidController.setI(float(self.kI))
        if self.kD.hasChanged():
            self.turnPidController.setD(float(self.kD))
        if self.tolerenceDegrees.hasChanged():
            self.turnPidController.setTolerance(positionTolerance=float(self.tolerenceDegrees))

        self.turnPidController.setSetpoint(
            getTargetRotation(self.robotContainer.odometry.getPose().translation()).degrees()
        )

        if not self.turnPidController.atSetpoint():
            self.tolerenceTimer.reset()

        if self.linearXSupplier is not None:
            speeds = WheelSpeedsPercentage.fromArcade(
                self.linearXProfiler.calculate(self.linearXSupplier()),
                0.0
            )

        if abs(self.turnPidController.getPositionError()) >= float(self.integralMaxError):
            self.turnPidController.setI(0.0)
        else:
            self.turnPidController.setI(float(self.kI))

        # Adjust drivetrain to aim at the hub
        turnSpeed = self.turnPidController.calculate(self.robotContainer.odometry.getPose().rotation().degrees())
        if abs(turnSpeed) < float(self.minVelocity):
            turnSpeed = math.copysign(float(self.minVelocity), turnSpeed)

        finalSpeeds = speeds + WheelSpeedsPercentage.fromArcade(
            -self.robotContainer.driverController.getLeftY(), -turnSpeed)
        self.robotContainer.robotDrive.tankDrive(finalSpeeds.left, finalSpeeds.right)

    def isFinished(self):
        if self.shouldAutoTerminate:
            return self.tolerenceTimer.hasPeriodPassed(self.tolerenceTime.getDefault())
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.tolerenceTimer.stop()
