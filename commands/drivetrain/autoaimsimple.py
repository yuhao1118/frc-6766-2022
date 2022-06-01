from commands2 import CommandBase
from wpilib import Timer
from wpimath.controller import PIDController

from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.axisProfile import AxisProfile
from lib.utils.tunablenumber import TunableNumber


class AutoAimSimple(CommandBase):
    """
    自瞄指令(简单)

    输入:
        robotContainer: RobotContainer实例
        shouldAutoTerminate: 是否自动终止
    """

    def __init__(self, robotContainer, io=None, shouldAutoTerminate=False):
        super().__init__()
        super().setName("AutoAimSimple")
        self.robotContainer = robotContainer

        self.linearXProfiler = None
        self.linearXSupplier = None

        if io is not None:
            self.linearXProfiler = AxisProfile(0.04)
            self.linearXSupplier = io.getDriveXSupplier()

        self.kP = TunableNumber("AutoAimSimple/kP", 0.007)
        self.kI = TunableNumber("AutoAimSimple/kI", 0.0)
        self.kD = TunableNumber("AutoAimSimple/kD", 0.0005)
        self.tolerenceDegrees = TunableNumber("AutoAimSimple/tolerenceDegrees", 3.0)
        self.tolerenceTime = TunableNumber("AutoAimSimple/toleranceTime", 1.0)
        self.shouldAutoTerminate = shouldAutoTerminate

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )
        self.turnPidController.setTolerance(positionTolerance=self.tolerenceDegrees.getDefault())
        self.turnPidController.enableContinuousInput(-180.0, 180.0)
        self.turnPidController.setSetpoint(0.0)

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

        if not self.turnPidController.atSetpoint():
            self.tolerenceTimer.reset()

        if self.linearXSupplier is not None:
            speeds = WheelSpeedsPercentage.fromArcade(
                self.linearXProfiler.calculate(self.linearXSupplier()),
                0.0
            )

        # Adjust drivetrain to aim at the hub
        turnSpeed = 0.0
        if self.robotContainer.visionControl.hasTargets():
            xOffset = self.robotContainer.visionControl.getXOffset().degrees()
            turnSpeed = self.turnPidController.calculate(xOffset)
        else:
            self.tolerenceTimer.reset()
            self.turnPidController.reset()

        finalSpeeds = speeds + WheelSpeedsPercentage.fromArcade(0.0, -turnSpeed)
        self.robotContainer.robotDrive.tankDrive(finalSpeeds.left, finalSpeeds.right)

    def isFinished(self):
        if self.shouldAutoTerminate:
            return self.tolerenceTimer.hasPeriodPassed(self.tolerenceTime.getDefault())
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.tolerenceTimer.stop()
