from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.controller import SimpleMotorFeedforwardMeters

import ctre
import constants


class Shooter(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.shooter = ctre.TalonFX(constants.kShooter)
        self.shooter.configFactoryDefault()
        self.shooter.setNeutralMode(ctre.NeutralMode.Coast)
        self.shooter.setInverted(constants.kShooterRotate)
        self.shootSpeed = 10

        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVoltsShooter,
            constants.kvVoltSecondsPerMeterShooter,
            constants.kaVoltSecondsSquaredPerMeterShooter
        )

        self.resetEncoder()


    def log(self):
        SmartDashboard.putNumber("Shooter Speed", self.getShooterEncoderSpeed())

    def periodic(self):
        self.log()
        self.shootSpeed = SmartDashboard.getNumber("ShooterV", 0)

    def setVolts(self, outputVolts):
        self.shooter.set(ctre.ControlMode.PercentOutput, outputVolts / 12)

    def setVelocity(self, outputVelocity):
        ff = self.feedforwardController.calculate(outputVelocity)
        self.setVolts(ff)

    def resetEncoder(self):
        self.shooter.setSelectedSensorPosition(0, 0, 20)

    def getShooterEncoderSpeed(self):
        return self.shooter.getSelectedSensorVelocity() * constants.kShooterEncoderDistancePerPulse * 10
    
    