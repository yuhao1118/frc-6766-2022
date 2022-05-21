from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController

import ctre
import constants
from lib.utils.tunablenumber import TunableNumber


class Shooter(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.shooter = ctre.TalonFX(constants.kShooterPort)
        self.shooter.configFactoryDefault()
        self.shooter.setNeutralMode(ctre.NeutralMode.Brake)
        self.shooter.setInverted(constants.kShooterRotate)
        self.shooter.configVoltageCompSaturation(constants.kNominalVoltage)
        self.shooter.enableVoltageCompensation(True)

        self.kP = TunableNumber("Shooter/kP", 0.04)
        self.kI = TunableNumber("Shooter/kI", 0.00)
        self.kD = TunableNumber("Shooter/kD", 2.00)
        self.kF = TunableNumber("Shooter/kF", 0.046)

        self.shooter.config_kP(0, self.kP.getDefault(), 0)
        self.shooter.config_kI(0, self.kI.getDefault(), 0)
        self.shooter.config_kD(0, self.kD.getDefault(), 0)
        self.shooter.config_kF(0, self.kF.getDefault(), 0)


    def log(self):
        SmartDashboard.putData("Shooter", self)
        SmartDashboard.putData("Shooter PID", self.pidController)
        SmartDashboard.putNumber("Shooter Speed", self.getShooterEncoderSpeed())

    def periodic(self):
        # self.log()
        if self.kP.hasChanged(): self.shooter.config_kP(0, float(self.kP), 0)
        if self.kI.hasChanged(): self.shooter.config_kI(0, float(self.kI), 0)
        if self.kD.hasChanged(): self.shooter.config_kD(0, float(self.kD), 0)
        if self.kF.hasChanged(): self.shooter.config_kF(0, float(self.kF), 0)

    def setVolts(self, outputVolts):
        self.shooter.set(ctre.ControlMode.PercentOutput, outputVolts / 12)

    def setRPS(self, rps):
        self.shooter.set(ctre.ControlMode.Velocity, rps / constants.kShooterEncoderRotatePerPulse / 10)


    def getShooterEncoderSpeed(self):
        return self.shooter.getSelectedSensorVelocity() * constants.kShooterEncoderDistancePerPulse * 10
    
    