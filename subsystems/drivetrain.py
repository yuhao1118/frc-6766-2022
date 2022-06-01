import math

from commands2 import SubsystemBase, RamseteCommand, InstantCommand, SequentialCommandGroup

from wpilib import SerialPort, SmartDashboard, Field2d, DriverStation
from wpimath.kinematics import DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, SimpleMotorFeedforwardMeters, PIDController
import ctre

import constants
from lib.sensors.wit_imu import WitIMU
from lib.utils.tunablenumber import TunableNumber
from lib.drivetrain.differentialdriveodometry import DifferentialVOdometry


class Drivetrain(SubsystemBase):

    def __init__(self):
        super().__init__()
        self.kP = TunableNumber("Drivetrain/kP", 0.78)
        self.kI = TunableNumber("Drivetrain/kI", 0.0)
        self.kD = TunableNumber("Drivetrain/kD", 0.0)
        self.brakeMode = True

        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVolts,
            constants.kvVoltSecondsPerMeter,
            constants.kaVoltSecondsSquaredPerMeter,
        )
        self.leftPIDController = PIDController(self.kP.getDefault(), self.kI.getDefault(), self.kD.getDefault())
        self.rightPIDController = PIDController(self.kP.getDefault(), self.kI.getDefault(), self.kD.getDefault())

        self.field2d = Field2d()  # Display field image in dashboard

        self.LF_motor = ctre.WPI_TalonFX(constants.kLeftMotor1Port)
        self.LR_motor = ctre.WPI_TalonFX(constants.kLeftMotor2Port)
        self.RF_motor = ctre.WPI_TalonFX(constants.kRightMotor1Port)
        self.RR_motor = ctre.WPI_TalonFX(constants.kRightMotor2Port)

        for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            motor.configFactoryDefault()
            motor.setNeutralMode(ctre.NeutralMode.Coast)
            motor.configVoltageCompSaturation(constants.kNominalVoltage)
            motor.enableVoltageCompensation(True)
            motor.configPeakOutputForward(constants.kDrivetrainMaxOutput)
            motor.configPeakOutputReverse(-constants.kDrivetrainMaxOutput)
            # motor.config_kP(0, self.kP.getDefault())
            # motor.config_kI(0, self.kI.getDefault())
            # motor.config_kD(0, self.kD.getDefault())
            # motor.config_kF(0, 0.0)
            # motor.configClosedloopRamp(0.4)

        self.LR_motor.follow(self.LF_motor)
        self.RR_motor.follow(self.RF_motor)

        self.LF_motor.setInverted(constants.kLeftMotorRotate)
        self.LR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)
        self.RF_motor.setInverted(constants.kRightMotorRotate)
        self.RR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)

        self.resetEncoder()

        self.gyro = WitIMU(SerialPort.Port.kUSB)
        self.gyro.calibrate()

        self.odometry = DifferentialVOdometry(self.gyro.getRotation2d())
        SmartDashboard.putData("Field2d", self.field2d)

    def log(self):
        SmartDashboard.putNumber("Left Encoder Speed", self.getLeftEncoderSpeed())
        SmartDashboard.putNumber("Right Encoder Speed", self.getRightEncoderSpeed())
        SmartDashboard.putNumber("Heading", self.gyro.getAngle())

    def periodic(self):
        if self.kP.hasChanged():
            # for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            #     motor.config_kP(0, float(self.kP))
            self.leftPIDController.setP(float(self.kP))
            self.rightPIDController.setP(float(self.kP))

        if self.kI.hasChanged():
            # for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            #     motor.config_kI(0, float(self.kI))
            self.leftPIDController.setI(float(self.kI))
            self.rightPIDController.setI(float(self.kI))

        if self.kD.hasChanged():
            # for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            #     motor.config_kD(0, float(self.kD))
            self.leftPIDController.setD(float(self.kD))
            self.rightPIDController.setD(float(self.kD))

        # Disable时容易推车
        if DriverStation.getInstance().isEnabled():
            if not self.getBrakeMode():
                self.setBrakeMode(True)
        else:
            if self.getBrakeMode():
                self.setBrakeMode(False)

        self.odometry.updatePose(
            self.gyro.getRotation2d(),
            math.radians(self.gyro.getRate()),
            self.getLeftEncoderDistance(),
            self.getRightEncoderDistance()
        )
        self.field2d.setRobotPose(self.getPose())
        self.log()

    def setBrakeMode(self, shouldBrake):
        self.brakeMode = shouldBrake
        for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            motor.setNeutralMode(ctre.NeutralMode.Brake if shouldBrake else ctre.NeutralMode.Coast)

    def getBrakeMode(self):
        return self.brakeMode

    def resetEncoder(self):
        self.LF_motor.setSelectedSensorPosition(0, 0, 20)
        self.LR_motor.setSelectedSensorPosition(0, 0, 20)
        self.RF_motor.setSelectedSensorPosition(0, 0, 20)
        self.RR_motor.setSelectedSensorPosition(0, 0, 20)

    def resetOdometry(self, pose):
        self.resetEncoder()
        self.odometry.resetPosition(pose, self.gyro.getRotation2d())

    def zeroHeading(self):
        self.gyro.reset()

    def tankDrive(self, leftPercentage, rightPercentage):
        self.LF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, leftPercentage)
        self.RF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, rightPercentage)

    def tankDriveVolts(self, leftVolts, rightVolts):
        self.tankDrive(leftVolts / 12, rightVolts / 12)

    # def tankDriveVelocity(self, leftVelocity, rightVelocity):
    #     leftFF = self.feedforwardController.calculate(leftVelocity) / constants.kNominalVoltage
    #     rightFF = self.feedforwardController.calculate(rightVelocity) / constants.kNominalVoltage
    #     leftVelocity = leftVelocity / constants.kDrivetrainEncoderDistancePerPulse / 10
    #     rightVelocity = rightVelocity / constants.kDrivetrainEncoderDistancePerPulse / 10
    #
    #     self.LF_motor.set(
    #         ctre.TalonFXControlMode.Velocity, leftVelocity, ctre.DemandType.ArbitraryFeedForward, leftFF)
    #     self.RF_motor.set(
    #         ctre.TalonFXControlMode.Velocity, rightVelocity, ctre.DemandType.ArbitraryFeedForward, rightFF)

    def tankDriveVelocity(self, leftVelocity, rightVelocity):
        leftFF = self.feedforwardController.calculate(leftVelocity)
        rightFF = self.feedforwardController.calculate(rightVelocity)
        leftVolts = self.leftPIDController.calculate(self.getLeftEncoderSpeed(), leftVelocity) + leftFF
        rightVolts = self.rightPIDController.calculate(self.getRightEncoderSpeed(), rightVelocity) + rightFF
        self.tankDriveVolts(leftVolts, rightVolts)

    ############## Getter functions ##############
    def getTrajectoryCommand(self, trajectory, shouldInitPose=True):

        # Close loop RaseteCommand
        ramseteCommand = RamseteCommand(
            trajectory,
            self.getPose,
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            self.feedforwardController,
            constants.kDriveKinematics,
            self.getWheelSpeeds,
            self.leftPIDController,
            self.rightPIDController,
            self.tankDriveVolts,
            [self],
        )
        # ramseteCommand = RamseteCommand(
        #     trajectory,
        #     self.getPose,
        #     RamseteController(constants.kRamseteB, constants.kRamseteZeta),
        #     constants.kDriveKinematics,
        #     self.tankDriveVelocity,
        #     [self]
        # )

        def beforeStart():
            self.field2d.getObject("traj").setTrajectory(trajectory)
            if shouldInitPose:
                self.resetOdometry(trajectory.initialPose())

        if shouldInitPose:
            return SequentialCommandGroup(
                InstantCommand(beforeStart),
                ramseteCommand,
                InstantCommand(lambda: self.tankDriveVolts(0, 0))
            )

        return SequentialCommandGroup(
            ramseteCommand,
            InstantCommand(lambda: self.tankDriveVolts(0, 0))
        )

    def getPose(self):
        # return the estimated pose of the robot
        return self.odometry.getPose()

    def getOdometry(self):
        return self.odometry

    def getWheelSpeeds(self):
        speeds = DifferentialDriveWheelSpeeds(
            self.getLeftEncoderSpeed(),
            self.getRightEncoderSpeed()
        )
        return speeds

    def getLeftEncoderSpeed(self):
        return (self.LF_motor.getSelectedSensorVelocity() + self.LR_motor.getSelectedSensorVelocity()) / 2 \
               * constants.kDrivetrainEncoderDistancePerPulse * 10

    def getRightEncoderSpeed(self):
        return (self.RF_motor.getSelectedSensorVelocity() + self.RR_motor.getSelectedSensorVelocity()) / 2 \
               * constants.kDrivetrainEncoderDistancePerPulse * 10

    def getLeftEncoderDistance(self):
        return (self.LF_motor.getSelectedSensorPosition() + self.LR_motor.getSelectedSensorPosition()) / 2 \
               * constants.kDrivetrainEncoderDistancePerPulse

    def getRightEncoderDistance(self):
        return (self.RF_motor.getSelectedSensorPosition() + self.RR_motor.getSelectedSensorPosition()) / 2 \
               * constants.kDrivetrainEncoderDistancePerPulse
