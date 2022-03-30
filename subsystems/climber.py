from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.controller import PIDController

import ctre
import constants
class Climber(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.L_motor = ctre.TalonFX(constants.kLeftClimbMotorPort)
        self.R_motor = ctre.TalonFX(constants.kRightClimbMotorPort)

        self.LArm_motor = ctre.TalonFX(constants.kLeftClimbArmMotorPort)
        self.RArm_motor = ctre.TalonFX(constants.kRightClimbArmMotorPort)

        for motor in [self.L_motor, self.R_motor, self.LArm_motor, self.RArm_motor]:
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
            motor.configForwardSoftLimitThreshold(
                constants.kClimbMotorSoftLimitForward, 0)
            motor.configForwardSoftLimitEnable(True, 0)

            motor.configReverseSoftLimitThreshold(
                constants.kClimbMotorSoftLimitReverse, 0)
            motor.configReverseSoftLimitEnable(True, 0)

        # Set climb arm motor soft limits
        for motor in [self.LArm_motor, self.RArm_motor]:
            motor.configForwardSoftLimitThreshold(
                constants.kClimbArmMotorSoftLimitForward, 0)
            motor.configForwardSoftLimitEnable(True, 0)

            motor.configReverseSoftLimitThreshold(
                constants.kClimbArmMotorSoftLimitReverse, 0)
            motor.configReverseSoftLimitEnable(True, 0)

        self.L_motor.setInverted(constants.kLeftClimbMotorRotate)
        self.R_motor.setInverted(constants.kRightClimbMotorRotate)
        self.LArm_motor.setInverted(constants.kLeftCLimbArmMotorRotate)
        self.RArm_motor.setInverted(constants.kRightCLimbArmMotorRotate)

        self.ArmPIDController = PIDController(constants.kPClimbArm, constants.kIClimbArm, constants.kDClimbArm)

        self.resetEncoder()

    def log(self):
        SmartDashboard.putData("Climber", self)
        SmartDashboard.putNumber("Climb left", self.getLeftEncoderDistance())
        SmartDashboard.putNumber("Climb right", self.getRightEncoderDistance())
        SmartDashboard.putNumber("Climb Arm Degree", self.getArmEncoderDegrees())

    def periodic(self):
        self.log()
        pass

    def set(self, output, rightOutput=None):
        leftOutput, rightOutput = output, output
        if rightOutput is not None:
            rightOutput = rightOutput

        self.L_motor.set(ctre.ControlMode.PercentOutput, leftOutput)
        self.R_motor.set(ctre.ControlMode.PercentOutput, rightOutput)

    def setArm(self, output):
        # Clockwise is positive
        self.LArm_motor.set(ctre.ControlMode.PercentOutput, output)
        self.RArm_motor.set(ctre.ControlMode.PercentOutput, output)

    def resetEncoder(self):
        self.L_motor.setSelectedSensorPosition(0, 0, 20)
        self.R_motor.setSelectedSensorPosition(0, 0, 20)

    def getLeftEncoderDistance(self):
        return self.L_motor.getSelectedSensorPosition() * constants.kClimbEncoderDistancePerPulse

    def getRightEncoderDistance(self):
        return self.R_motor.getSelectedSensorPosition() * constants.kClimbEncoderDistancePerPulse

    def getArmEncoderDegrees(self):
        rawPulse = (self.LArm_motor.getSelectedSensorPosition() + self.RArm_motor.getSelectedSensorPosition()) / 2
        degrees = rawPulse * constants.kClimbArmEncoderDegreesPerPulse
        return degrees
