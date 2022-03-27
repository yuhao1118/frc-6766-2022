from commands2 import SubsystemBase
from wpilib import SmartDashboard

import ctre
import constants


class Climber(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.L_motor = ctre.TalonFX(constants.kLeftClimbMotorPort)
        self.R_motor = ctre.TalonFX(constants.kRightClimbMotorPort)

        self.L_motor.configFactoryDefault()
        self.R_motor.configFactoryDefault()

        self.L_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.R_motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.L_motor.setInverted(constants.kLeftClimbMotorRotate)
        self.R_motor.setInverted(constants.kRightClimbMotorRotate)

        self.resetEncoder()

    def log(self):
        SmartDashboard.putNumber("Climb left", self.getLeftEncoderDistance())
        SmartDashboard.putNumber("Climb right", self.getRightEncoderDistance())

    def periodic(self):
        self.log()

    def setVolts(self, outputVolts, rightOutputVolts=None):
        leftOutput, rightOutput = outputVolts, outputVolts
        if rightOutputVolts is not None:
            rightOutput = rightOutputVolts

        self.L_motor.set(ctre.ControlMode.PercentOutput, leftOutput / 12)
        self.R_motor.set(ctre.ControlMode.PercentOutput, rightOutput / 12)

    def resetEncoder(self):
        self.L_motor.setSelectedSensorPosition(0, 0, 20)
        self.R_motor.setSelectedSensorPosition(0, 0, 20)

    def getLeftEncoderDistance(self):
        return self.L_motor.getSelectedSensorPosition() * constants.kClimbEncoderDistancePerPulse

    def getRightEncoderDistance(self):
        return self.R_motor.getSelectedSensorPosition() * constants.kClimbEncoderDistancePerPulse