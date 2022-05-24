from commands2 import SubsystemBase
import ctre
import constants

class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.LArm_motor = ctre.TalonFX(constants.kLeftClimbArmMotorPort)
        self.RArm_motor = ctre.TalonFX(constants.kRightClimbArmMotorPort)

        for motor in [self.LArm_motor, self.RArm_motor]:
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

        # Set climb arm motor soft limits
        # for motor in [self.LArm_motor, self.RArm_motor]:
        #     motor.configForwardSoftLimitThreshold(
        #         constants.kClimbArmMotorSoftLimitForward, 0)
        #     motor.configForwardSoftLimitEnable(True, 0)

        #     motor.configReverseSoftLimitThreshold(
        #         constants.kClimbArmMotorSoftLimitReverse, 0)
        #     motor.configReverseSoftLimitEnable(True, 0)

        self.LArm_motor.setInverted(constants.kLeftCLimbArmMotorRotate)
        self.RArm_motor.setInverted(constants.kRightCLimbArmMotorRotate)

        self.RArm_motor.follow(self.LArm_motor)

    def log(self):
        SmartDashboard.putNumber("Climb Arm Degree", self.getArmEncoderDegrees())

    def periodic(self):
        # self.log()
        pass

    def set(self, output):
        self.LArm_motor.set(ctre.ControlMode.PercentOutput, output)

    def resetEncoder(self):
        self.LArm_motor.setSelectedSensorPosition(0, 0, 20)
        self.RArm_motor.setSelectedSensorPosition(0, 0, 20)

    def getArmEncoderDegrees(self):
        rawPulse = (self.LArm_motor.getSelectedSensorPosition() + self.RArm_motor.getSelectedSensorPosition()) / 2
        degrees = rawPulse * constants.kClimbArmEncoderDegreesPerPulse
        return degrees