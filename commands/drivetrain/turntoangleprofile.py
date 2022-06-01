from commands2 import CommandBase
from wpilib import Timer
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.tunablenumber import TunableNumber


class TurnToAngleProfile(CommandBase):
    """
    转向指定角度指令

    输入:
        robotContainer: RobotContainer实例
    """

    maxDegreesVelocityPerSecond = 360
    maxDegreesAccelerationPerSecond = 360

    def __init__(self, robotContainer):
        super().__init__()
        super().setName("DebugAutoAim")
        self.robotContainer = robotContainer
        self.kP = TunableNumber("AutoAim/kP", 0.0029)
        self.kI = TunableNumber("AutoAim/kI", 0.0)
        self.kD = TunableNumber("AutoAim/kD", 0.0005)
        self.integralMaxError = TunableNumber("AutoAim/IntegralMaxError", 0.0)
        self.tolerenceDegrees = TunableNumber("AutoAim/tolerenceDegrees", 3.0)
        self.tolerenceTime = TunableNumber("AutoAim/toleranceTime", 0.3)
        self.angle = TunableNumber("AutoAim/debugAngle", 90.0)
        self.initialDegrees = 0.0

        self.turnPidController = ProfiledPIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault(),
            TrapezoidProfile.Constraints(self.maxDegreesVelocityPerSecond, self.maxDegreesAccelerationPerSecond)
        )
        self.turnPidController.setTolerance(positionTolerance=self.tolerenceDegrees.getDefault())
        self.turnPidController.enableContinuousInput(-180.0, 180.0)

        self.tolerenceTimer = Timer()

        self.addRequirements(self.robotContainer.robotDrive)

    def initialize(self):
        self.turnPidController.reset(0.0)
        self.tolerenceTimer.reset()
        self.tolerenceTimer.start()
        self.initialDegrees = self.robotContainer.odometry.getPose().rotation().degrees()
        self.turnPidController.setGoal(self.initialDegrees + float(self.angle))

    def execute(self):
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

        if abs(self.turnPidController.getPositionError()) >= float(self.integralMaxError):
            self.turnPidController.setI(0.0)
        else:
            self.turnPidController.setI(float(self.kI))

        turnSpeed = self.turnPidController.calculate(self.robotContainer.odometry.getPose().rotation().degrees())
        speeds = WheelSpeedsPercentage.fromArcade(0.0, turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return self.tolerenceTimer.hasElapsed(self.tolerenceTime.get())

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
        self.tolerenceTimer.stop()
