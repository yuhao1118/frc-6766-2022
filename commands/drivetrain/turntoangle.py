from commands2 import CommandBase
from wpimath.controller import PIDController
from lib.utils.tunablenumber import TunableNumber
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage

import math


class TurnToAngleCommand(CommandBase):
    def __init__(self, robotContainer, angleRad):
        super().__init__()
        self.angle = angleRad
        self.robotContainer = robotContainer
        self.kP = TunableNumber("Turn/kP", 0.17)
        self.kI = TunableNumber("Turn/kI", 0.0)
        self.kD = TunableNumber("Turn/kD", 0.0)
        self.kF = TunableNumber("Turn/kF", 0.025)

        self.turnPidController = PIDController(
            self.kP.getDefault(),
            self.kI.getDefault(),
            self.kD.getDefault()
        )

        self.turnPidController.setTolerance(positionTolerance=0.035)
        self.turnPidController.enableContinuousInput(-math.pi, math.pi)
        self.addRequirements(self.robotContainer.robotDrive)

    def initialize(self):
        self.gyroOffset = self.robotContainer.robotDrive.gyro.getRotation2d().radians()
        self.setpoint = self.angle + self.gyroOffset

    def execute(self):
        if self.kP.hasChanged():
            self.turnPidController.setP(self.kP.get())
        if self.kI.hasChanged():
            self.turnPidController.setI(self.kI.get())
        if self.kD.hasChanged():
            self.turnPidController.setD(self.kD.get())

        turnSpeed = -self.turnPidController.calculate(
            self.robotContainer.robotDrive.gyro.getRotation2d().radians(), self.setpoint) + float(self.kF) * self.angle
        # turnSpeed = float(self.kF) * self.angle

        speeds = WheelSpeedsPercentage.fromArcade(0, turnSpeed)
        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrputed):
        self.robotContainer.robotDrive.tankDrive(0, 0)
