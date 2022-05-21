from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation, Timer

import ctre
import constants
class Elevator(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.L_motor = ctre.TalonFX(constants.kLeftClimbMotorPort)
        self.R_motor = ctre.TalonFX(constants.kRightClimbMotorPort)

        for motor in [self.L_motor, self.R_motor]:
            motor.configFactoryDefault()
            motor.setNeutralMode(ctre.NeutralMode.Brake)
            motor.configVoltageCompSaturation(constants.kNominalVoltage)
            motor.enableVoltageCompensation(True)
            motor.configStatorCurrentLimit(ctre.StatorCurrentLimitConfiguration(
                True,
                constants.kClimbMotorCurrentLimit,
                constants.kClimbMotorThresholdCurrent,
                constants.kClimbMotorThresholdDuration
            ))
            motor.configSupplyCurrentLimit(ctre.SupplyCurrentLimitConfiguration(
                True,
                constants.kClimbMotorCurrentLimit,
                constants.kClimbMotorThresholdCurrent,
                constants.kClimbMotorThresholdDuration
            ))

        # Set climb motor soft limits
        for motor in [self.L_motor, self.R_motor]:
            motor.configReverseSoftLimitThreshold(
                constants.kClimbMotorSoftLimitReverse, 0)
        

        self.L_motor.setInverted(constants.kLeftClimbMotorRotate)
        self.R_motor.setInverted(constants.kRightClimbMotorRotate)

        self.resetActive = False
        self.resetComplete = False
        self.resetGraceTimer = Timer()

        self.resetGraceTimer.start()

    def log(self):
        SmartDashboard.putData("Elevator", self)
        SmartDashboard.putNumber("Elevator Distance", self.getClimbEncoderDistance())
        SmartDashboard.putNumber("Elevator Speed", self.getClimbEncoderSpeed())
    
    def periodic(self):
        # self.log()
        if not self.resetComplete:
            if DriverStation.getInstance().isEnabled():
                if not self.resetActive:
                    self.resetActive = True
                    self.resetGraceTimer.reset()
                    self.set(0.50)
                else:
                    vel = self.getClimbEncoderSpeed()
                    if self.resetGraceTimer.hasElapsed(0.2) and abs(vel) < 8000:
                        self.resetComplete = True
                        self.set(0)
                        self.resetActive = False
                        self.resetGraceTimer.reset()
                        self.resetEncoder()


    def set(self, output, rightOutput=None):
        leftOutput, rightOutput = output, output
        if rightOutput is not None:
            rightOutput = rightOutput

        self.L_motor.set(ctre.ControlMode.PercentOutput, leftOutput)
        self.R_motor.set(ctre.ControlMode.PercentOutput, rightOutput)

    def resetEncoder(self):
        self.L_motor.setSelectedSensorPosition(0, 0, 20)
        self.R_motor.setSelectedSensorPosition(0, 0, 20)

        self.L_motor.configReverseSoftLimitEnable(True)
        self.R_motor.configReverseSoftLimitEnable(True)

    def reset(self):
        self.L_motor.configReverseSoftLimitEnable(False)
        self.R_motor.configReverseSoftLimitEnable(False)

        self.resetComplete = False

    def getClimbEncoderDistance(self):
        return (self.L_motor.getSelectedSensorPosition() + self.R_motor.getSelectedSensorPosition()) / 2

    def getClimbEncoderSpeed(self):
        return (self.L_motor.getSelectedSensorVelocity() + self.R_motor.getSelectedSensorVelocity()) / 2
