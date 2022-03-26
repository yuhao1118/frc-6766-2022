from commands2 import SubsystemBase, RamseteCommand, InstantCommand, WaitCommand, SequentialCommandGroup

from wpilib import SerialPort, SmartDashboard, Field2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters

import constants
import ctre
from sensor.wit_imu import WitIMU


class Drivetrain(SubsystemBase):

    def __init__(self):

        super().__init__()
        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVolts,
            constants.kvVoltSecondsPerMeter,
            constants.kaVoltSecondsSquaredPerMeter,
        )

        self.field2d = Field2d()    # Display field image in dashboard

        self.LF_motor = ctre.TalonFX(constants.kLeftMotor1Port)
        self.LR_motor = ctre.TalonFX(constants.kLeftMotor2Port)
        self.RF_motor = ctre.TalonFX(constants.kRightMotor1Port)
        self.RR_motor = ctre.TalonFX(constants.kRightMotor2Port)

        self.LF_motor.configFactoryDefault()
        self.LR_motor.configFactoryDefault()
        self.RF_motor.configFactoryDefault()
        self.RR_motor.configFactoryDefault()

        self.LF_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.LR_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.RF_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.RR_motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.LR_motor.follow(self.LF_motor)
        self.RR_motor.follow(self.RF_motor)

        self.LF_motor.setInverted(constants.kLeftMotorRotate)
        self.LR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)
        self.RF_motor.setInverted(constants.kRightMotorRotate)
        self.RR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)

        self.LF_motor.configOpenloopRamp(constants.kOpenloopRampRate, 20)
        self.LR_motor.configOpenloopRamp(constants.kOpenloopRampRate, 20)
        self.RF_motor.configOpenloopRamp(constants.kOpenloopRampRate, 20)
        self.RR_motor.configOpenloopRamp(constants.kOpenloopRampRate, 20)

        self.setMaxOutput(0.75)
        self.resetEncoder()

        self.gyro = WitIMU(SerialPort.Port.kUSB)
        self.gyro.calibrate()

        self.odometry = DifferentialDriveOdometry(self.gyro.getRotation2d())

        self.configPID(constants.kP, constants.kI, constants.kD)

    def log(self):
        SmartDashboard.putData("Field2d", self.field2d)
        SmartDashboard.putData("LeftPIDController", self.leftPIDController)
        SmartDashboard.putData("RightPIDController", self.rightPIDController)
        SmartDashboard.putNumber("Left Encoder Speed",
                                 self.getLeftEncoderSpeed())
        SmartDashboard.putNumber(
            "Right Encoder Speed", self.getRightEncoderSpeed())
        SmartDashboard.putNumber(
            "Left Encoder Distance", self.getLeftEncoderDistance())
        SmartDashboard.putNumber(
            "Right Encoder Distance", self.getRightEncoderDistance())
        SmartDashboard.putNumber("Heading", self.gyro.getAngle())
        SmartDashboard.putNumber("Gyro Rate", self.gyro.getRate())

    def periodic(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.getLeftEncoderDistance(),
            self.getRightEncoderDistance()
        )
        self.field2d.setRobotPose(self.getPose())
        self.log()

    def resetEncoder(self):
        self.LF_motor.setSelectedSensorPosition(0, 0, 20)
        self.LR_motor.setSelectedSensorPosition(0, 0, 20)
        self.RF_motor.setSelectedSensorPosition(0, 0, 20)
        self.RR_motor.setSelectedSensorPosition(0, 0, 20)

    def resetOdometry(self, pose):
        self.resetEncoder()
        self.odometry.resetPosition(pose, self.gyro.getRotation2d())

    def tankDrive(self, leftPercentage, rightPercentage):
        self.LF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, leftPercentage)
        self.RF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, rightPercentage)

    def tankDriveVelocity(self, leftVelocity, rightVelocity):
        # Convert velocity in native Talon encoder units per 100ms.
        leftVelocity = leftVelocity / constants.kDriveTrainEncoderDistancePerPulse / 10
        rightVelocity = rightVelocity / constants.kDriveTrainEncoderDistancePerPulse / 10

        # Calculate the feedforward voltage and normalised in [-1, 1].
        leftFF = self.feedforwardController.calculate(
            leftVelocity) / constants.kNominalVoltage
        rightFF = self.feedforwardController.calculate(
            rightVelocity) / constants.kNominalVoltage

        # The first demand is the velocity (in native unit) setpoint.
        # When specifying the second demand type to be ArbitraryFeedForward, it will just
        # takes any compensated value ranging from -1 to 1. So that we can feed it with the
        # (normalised) feedforward voltages.
        self.LF_motor.set(ctre.TalonFXControlMode.Velocity, leftVelocity,
                          ctre.TalonFXDemandType.ArbitraryFeedForward, leftFF)
        self.RF_motor.set(ctre.TalonFXControlMode.Velocity, rightVelocity,
                          ctre.TalonFXDemandType.ArbitraryFeedForward, rightFF)

    def arcadeDrive(self, throttle, turn):
        if abs(throttle) < 0.05:
            throttle = 0

        if abs(turn) < 0.05:
            turn = 0

        self.tankDrive(throttle + turn, throttle - turn)

    def zeroHeading(self):
        self.gyro.reset()

    def setMaxOutput(self, maxOutput):
        self.LF_motor.configPeakOutputForward(maxOutput, 20)
        self.RF_motor.configPeakOutputForward(maxOutput, 20)
        self.LF_motor.configPeakOutputReverse(-maxOutput, 20)
        self.RF_motor.configPeakOutputReverse(-maxOutput, 20)

    def configPID(self, kP, kI, kD):
        for motor in [self.LF_motor, self.RF_motor]:
            motor.configSelectedFeedbackSensor(
                ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 20)

            motor.config_kP(0, kP, 20)
            motor.config_kI(0, kI, 20)
            motor.config_kD(0, kD, 20)

        # Stop motors
        self.tankDriveVelocity(0, 0)

    ############## Getter functions ##############
    def getTrajetoryCommand(self, trajectory, shouldInitPose=True):
        # Ref: https://www.chiefdelphi.com/t/feedforward-for-talonfx-pid/401200/8
        #
        # It is indeed a close-loop RamseteCommand althrough it does not look like. What we do
        # here is to apply on-falcon pid control + the on-rio feedforward control to follow a trajectory.
        # Developer must be careful about the unit conversion, especially the velocity which native Talon
        # measures it in 100ms instead of 1s.
        #
        # The benefits (instead of doing it all on-rio nor on-falcon) are:
        # 1. The on-falcon PID reacts faster and is easy to tune (using Phoenix Tuner).
        # 2. The on-rio feedforward control (namely the SimpleMotorFeedforwardMeters) could compensate the
        # for the static friction, which only setting kF is insufficient. We think the static force compensation
        # is important for the drivetrain but not for other subsystems.

        ramseteCommand = RamseteCommand(
            trajectory,
            self.getPose,
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            constants.kDriveKinematics,
            self.tankDriveVelocity,
            [self],
        )

        if shouldInitPose:
            return SequentialCommandGroup(
                InstantCommand(lambda: self.resetOdometry(
                    trajectory.initialPose())),
                ramseteCommand,
                InstantCommand(lambda: self.tankDriveVelocity(0, 0))
            )

        return SequentialCommandGroup(
            ramseteCommand,
            InstantCommand(lambda: self.tankDriveVelocity(0, 0))
        )

    def getPose(self):
        # return the estimated pose of the robot
        return self.odometry.getPose()

    def getWheelSpeeds(self):
        speeds = DifferentialDriveWheelSpeeds(
            self.getLeftEncoderSpeed(),
            self.getRightEncoderSpeed()
        )
        return speeds

    def getLeftEncoderSpeed(self):
        return self.LF_motor.getSelectedSensorVelocity() * constants.kDriveTrainEncoderDistancePerPulse * 10

    def getRightEncoderSpeed(self):
        return self.RF_motor.getSelectedSensorVelocity() * constants.kDriveTrainEncoderDistancePerPulse * 10

    def getLeftEncoderDistance(self):
        return self.LF_motor.getSelectedSensorPosition() * constants.kDriveTrainEncoderDistancePerPulse

    def getRightEncoderDistance(self):
        return self.RF_motor.getSelectedSensorPosition() * constants.kDriveTrainEncoderDistancePerPulse
