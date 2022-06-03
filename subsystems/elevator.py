from enum import Enum

from wpilib import SmartDashboard, DriverStation
from commands2 import FunctionalCommand, StartEndCommand, InstantCommand
from lib.statefulsubsystem import StatefulSubsystem

import ctre
import constants


class ElevatorState(Enum):
    IDLE = "idle"
    EXTENDING = "extending"
    RETRACTING = "retracting"
    HOLDING = "holding"
    RESETTING = "resetting"


class Elevator(StatefulSubsystem):

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

        self.isLeftFwdSwitchClose, self.isRightFwdSwitchClose = False, False
        self.initStates()
        SmartDashboard.putData("Elevator", self)

    def initStates(self):
        self.addStateAction(ElevatorState.IDLE, InstantCommand(
            lambda: self.set(0.0),
            [self]
        ), default=True)
        self.addStateAction(ElevatorState.EXTENDING, StartEndCommand(
            lambda: self.set(-1.0),
            lambda: self.set(0.0),
            [self]
        ))
        self.addStateAction(ElevatorState.RETRACTING, StartEndCommand(
            lambda: self.set(1.0),
            lambda: self.set(0.0),
            [self]
        ))
        self.addStateAction(ElevatorState.HOLDING, StartEndCommand(
            lambda: self.set(0.08),
            lambda: self.set(0.0),
            [self]
        ))
        self.addStateAction(ElevatorState.RESETTING, FunctionalCommand(
            self.resetInit,
            self.resetExecute,
            self.resetEnd,
            self.isRetracted,
            [self]
        ))
        super().initStates()

    def log(self):
        SmartDashboard.putNumber("Elevator Distance", self.getClimbEncoderDistance())
        SmartDashboard.putNumber("Elevator Speed", self.getClimbEncoderSpeed())

    def periodic(self):
        # self.log()
        self.isLeftFwdSwitchClose = bool(self.L_motor.isFwdLimitSwitchClosed())
        self.isRightFwdSwitchClose = bool(self.R_motor.isFwdLimitSwitchClosed())

        if self.isRetracted():
            self.resetEncoder()

        if self.hasChanged():
            print("Switch to", self.getState())
            self.getStateAction(self.getState()).schedule()

    def set(self, output, rightOutput=None):
        print("Elevator set", output, rightOutput)
        _leftOutput, _rightOutput = output, output

        if rightOutput is not None:
            _rightOutput = rightOutput

        self.L_motor.set(ctre.ControlMode.PercentOutput, _leftOutput)
        self.R_motor.set(ctre.ControlMode.PercentOutput, _rightOutput)

    def resetEncoder(self):
        self.L_motor.setSelectedSensorPosition(0, 0, 20)
        self.R_motor.setSelectedSensorPosition(0, 0, 20)

    def resetInit(self):
        self.L_motor.configReverseSoftLimitEnable(False)
        self.R_motor.configReverseSoftLimitEnable(False)
        if DriverStation.getInstance().isEnabled():
            self.set(0.5)

    def resetExecute(self):
        if DriverStation.getInstance().isEnabled():
            if self.isLeftFwdSwitchClose and not self.isRightFwdSwitchClose:
                self.set(0.0, 0.5)
            elif not self.isLeftFwdSwitchClose and self.isRightFwdSwitchClose:
                self.set(0.5, 0.0)

    def isRetracted(self):
        return DriverStation.getInstance().isDisabled() or (self.isLeftFwdSwitchClose and self.isRightFwdSwitchClose)

    def resetEnd(self, interrupted):
        self.setState(ElevatorState.IDLE)
        self.resetEncoder()
        self.L_motor.configReverseSoftLimitEnable(True)
        self.R_motor.configReverseSoftLimitEnable(True)

    def getClimbEncoderDistance(self):
        return (self.L_motor.getSelectedSensorPosition() + self.R_motor.getSelectedSensorPosition()) / 2

    def getClimbEncoderSpeed(self):
        return (self.L_motor.getSelectedSensorVelocity() + self.R_motor.getSelectedSensorVelocity()) / 2
