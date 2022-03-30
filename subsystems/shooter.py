from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController

import ctre
import constants


class Shooter(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.shooter = ctre.TalonFX(constants.kShooter)
        self.shooter.configFactoryDefault()
        self.shooter.setNeutralMode(ctre.NeutralMode.Brake)
        self.shooter.setInverted(constants.kShooterRotate)
        self.shooter.configVoltageCompSaturation(constants.kNominalVoltage)
        self.shooter.enableVoltageCompensation(True)

        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVoltsShooter,
            constants.kvVoltSecondsPerMeterShooter,
            constants.kaVoltSecondsSquaredPerMeterShooter
        )
        self.pidController = PIDController(
            constants.kPShooter,
            constants.kIShooter,
            constants.kDShooter,
        )

        self.resetEncoder()


    def log(self):
        SmartDashboard.putData("Shooter", self)
        SmartDashboard.putData("Shooter PID", self.pidController)
        SmartDashboard.putNumber("Shooter Speed", self.getShooterEncoderSpeed())

    def periodic(self):
        # self.log()
        pass

    def setVolts(self, outputVolts):
        self.shooter.set(ctre.ControlMode.PercentOutput, outputVolts / 12)

    def setVelocity(self, outputVelocity):
        ff = self.feedforwardController.calculate(outputVelocity)
        bf = self.pidController.calculate(self.getShooterEncoderSpeed(), outputVelocity)
        self.setVolts(ff + bf)

    def resetEncoder(self):
        self.shooter.setSelectedSensorPosition(0, 0, 20)

    def getShooterEncoderSpeed(self):
        return self.shooter.getSelectedSensorVelocity() * constants.kShooterEncoderDistancePerPulse * 10
    
    