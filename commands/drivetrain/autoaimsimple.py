from commands2 import CommandBase
from wpilib import Timer
from wpimath.controller import PIDController

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.tunablenumber import TunableNumber
from lib.utils.maths import clamp, axisProfile


class AutoAimSimple(CommandBase):
    """
    自瞄指令(简单)

    输入:
        robotContainer: RobotContainer实例
        controller=None: 手柄控制器, 默认None
    """

    def __init__(self, robotContainer, controller=None):
        super().__init__()
        super().setName("AutoAimSimple")
        self.robotContainer = robotContainer
        self.kP = TunableNumber("AutoAimSimple/kP", 0.005)
        self.kI = TunableNumber("AutoAimSimple/kI", 0.0)
        self.kD = TunableNumber("AutoAimSimple/kD", 0.0)
        self.tolerenceDegrees = TunableNumber("AutoAimSimple/tolerenceDegrees", 3.0)
        self.tolerenceTime = TunableNumber("AutoAimSimple/toleranceTime", 0.3)

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )
        self.turnPidController.setTolerance(positionTolerance=self.tolerenceDegrees.getDefault())
        self.turnPidController.enableContinuousInput(-180.0, 180.0)
        self.turnPidController.setSetpoint(0.0)

        self.tolerenceTimer = Timer()
        self.controller = controller

        self.addRequirements(self.robotContainer.robotDrive)

    def initialize(self):
        self.robotContainer.visionControl.setPipeline(0)
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

        if not self.turnPidController.atSetpoint():
            self.tolerenceTimer.reset()

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
        turnSpeed = 0.0
        if self.robotContainer.visionControl.hasTargets():
            xOffset = self.robotContainer.visionControl.getXOffset().degrees()
            turnSpeed = self.turnPidController.calculate(xOffset)
        else:
            self.tolerenceTimer.reset()
            self.turnPidController.reset()

        speeds = speeds + WheelSpeedsPercentage.fromArcade(0.0, -turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return self.tolerenceTimer.hasElapsed(self.tolerenceTime.get())

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.robotContainer.visionControl.setPipeline(1)
        self.tolerenceTimer.stop()
