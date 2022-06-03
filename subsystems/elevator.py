from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation, Timer
from lib.utils.tunablenumber import TunableNumber

import ctre
import constants


class Elevator(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.L_motor = ctre.TalonFX(constants.kLeftClimbMotorPort)
        self.R_motor = ctre.TalonFX(constants.kRightClimbMotorPort)

        self.L_motor.configFactoryDefault()
        self.R_motor.configFactoryDefault()

        self.L_motor.setInverted(constants.kLeftClimbMotorRotate)
        self.R_motor.setInverted(constants.kRightClimbMotorRotate)

        for motor in [self.L_motor, self.R_motor]:
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
            # 反向限位是软限位，是否启用由resetComplete标志决定
            motor.configReverseSoftLimitThreshold(constants.kClimbMotorSoftLimitReverse)

            # 正向限位是限位开关，默认启用
            motor.configForwardLimitSwitchSource(ctre.LimitSwitchSource.FeedbackConnector,
                                                 ctre.LimitSwitchNormal.NormallyOpen)
            motor.overrideLimitSwitchesEnable(True)

        self.resetActive = False
        self.resetComplete = True
        self.resetGraceTimer = Timer()

        self.resetGraceTimer.start()

        self.holdActive = False
        self.holdPosition = 0.0

        SmartDashboard.putData("Elevator", self)

    def log(self):
        SmartDashboard.putNumber("Elevator Distance", self.getClimbEncoderDistance())
        SmartDashboard.putNumber("Elevator Speed", self.getClimbEncoderSpeed())

    def periodic(self):
        # self.log()
        if not self.resetComplete:
            if DriverStation.getInstance().isEnabled():
                if not self.resetActive:
                    self.resetActive = True
                    self.resetGraceTimer.reset()
                    self.set(0.5)
                else:
                    if bool(self.L_motor.isFwdLimitSwitchClosed()) and not bool(self.R_motor.isFwdLimitSwitchClosed()):
                        self.set(0.0, 0.5)

                    if not bool(self.L_motor.isFwdLimitSwitchClosed()) and bool(self.R_motor.isFwdLimitSwitchClosed()):
                        self.set(0.5, 0.0)

                    if bool(self.L_motor.isFwdLimitSwitchClosed()) and bool(self.R_motor.isFwdLimitSwitchClosed()):
                        self.set(0.0, 0.0)
                        self.resetComplete = True
                        self.resetActive = False
                        self.resetGraceTimer.reset()
                        self.resetEncoder()

    def set(self, output, rightOutput=None):
        self.holdActive = False

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
