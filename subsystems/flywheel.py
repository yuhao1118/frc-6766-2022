from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController

import ctre
import constants
from lib.utils.tunablenumber import TunableNumber


class Flywheel(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.shooter = ctre.TalonFX(constants.kShooterPort)
        self.shooter.configFactoryDefault()
        self.shooter.setNeutralMode(ctre.NeutralMode.Coast)
        self.shooter.setInverted(constants.kShooterRotate)
        self.shooter.configVoltageCompSaturation(constants.kNominalVoltage)
        self.shooter.enableVoltageCompensation(True)

        self.kP = TunableNumber("Flywheel/kP", 0.04)
        self.kI = TunableNumber("Flywheel/kI", 0.00)
        self.kD = TunableNumber("Flywheel/kD", 2.00)
        self.kF = TunableNumber("Flywheel/kF", 0.046)

        self.shooter.config_kP(0, self.kP.getDefault(), 0)
        self.shooter.config_kI(0, self.kI.getDefault(), 0)
        self.shooter.config_kD(0, self.kD.getDefault(), 0)
        self.shooter.config_kF(0, self.kF.getDefault(), 0)

        self.setActive = False

        self.flywheelReady = False

    def log(self):
        SmartDashboard.putData("Flywheel", self)
        SmartDashboard.putData("Flywheel PID", self.pidController)
        SmartDashboard.putNumber(
            "Flywheel Speed", self.getShooterEncoderSpeed())

    def periodic(self):
        # self.log()
        if self.kP.hasChanged():
            self.shooter.config_kP(0, float(self.kP), 0)
        if self.kI.hasChanged():
            self.shooter.config_kI(0, float(self.kI), 0)
        if self.kD.hasChanged():
            self.shooter.config_kD(0, float(self.kD), 0)
        if self.kF.hasChanged():
            self.shooter.config_kF(0, float(self.kF), 0)

        if self.setActive and abs(self.shooter.getClosedLoopError()) < 500:
            self.flywheelReady = True

    def setRPS(self, rps):
        self.shooter.set(ctre.ControlMode.Velocity, rps /
                         constants.kShooterEncoderRotatePerPulse / 10)
        self.setActive = True

    def reset(self):
        self.setRPS(0.0)
        self.setActive = False

    def isFlywheelReady(self):
        return self.flywheelReady

    def getShooterEncoderSpeed(self):
        return self.shooter.getSelectedSensorVelocity() * constants.kShooterEncoderDistancePerPulse * 10
