import math
from commands2 import SubsystemBase, RamseteCommand, InstantCommand, SequentialCommandGroup

from wpilib import SerialPort, SmartDashboard, Field2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters
import ctre

import constants
from lib.sensors.wit_imu import WitIMU
class Drivetrain(SubsystemBase):

    def __init__(self):
        super().__init__()
        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVolts,
            constants.kvVoltSecondsPerMeter,
            constants.kaVoltSecondsSquaredPerMeter,
        )

        self.field2d = Field2d()    # Display field image in dashboard

        self.LF_motor = ctre.WPI_TalonFX(constants.kLeftMotor1Port)
        self.LR_motor = ctre.WPI_TalonFX(constants.kLeftMotor2Port)
        self.RF_motor = ctre.WPI_TalonFX(constants.kRightMotor1Port)
        self.RR_motor = ctre.WPI_TalonFX(constants.kRightMotor2Port)

        for motor in [self.LF_motor, self.LR_motor, self.RF_motor, self.RR_motor]:
            motor.configFactoryDefault()
            motor.setNeutralMode(ctre.NeutralMode.Brake)
            motor.configVoltageCompSaturation(constants.kNominalVoltage)
            motor.enableVoltageCompensation(True)
            motor.configPeakOutputForward(constants.kDrivetrainMaxOutput)
            motor.configPeakOutputReverse(-constants.kDrivetrainMaxOutput)

        self.LR_motor.follow(self.LF_motor)
        self.RR_motor.follow(self.RF_motor)

        self.LF_motor.setInverted(constants.kLeftMotorRotate)
        self.LR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)
        self.RF_motor.setInverted(constants.kRightMotorRotate)
        self.RR_motor.setInverted(ctre.TalonFXInvertType.FollowMaster)

        self.resetEncoder()
        self.setOpenloopRamp(constants.kOpenloopRampRateTeleop)

        self.gyro = WitIMU(SerialPort.Port.kUSB)
        self.gyro.calibrate()

        self.odometry = DifferentialDriveOdometry(self.gyro.getRotation2d())

        self.leftPIDController = PIDController(
            constants.kP, constants.kI, constants.kD)
        self.rightPIDController = PIDController(
            constants.kP, constants.kI, constants.kD)


    def log(self):
        SmartDashboard.putData("Drivetrain", self)
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
        # self.log()

    def setOpenloopRamp(self, seconds):
        self.LF_motor.configOpenloopRamp(seconds, 0)
        self.LR_motor.configOpenloopRamp(seconds, 0)
        self.RF_motor.configOpenloopRamp(seconds, 0)
        self.RR_motor.configOpenloopRamp(seconds, 0)

    def resetEncoder(self):
        self.LF_motor.setSelectedSensorPosition(0, 0, 0)
        self.LR_motor.setSelectedSensorPosition(0, 0, 0)
        self.RF_motor.setSelectedSensorPosition(0, 0, 0)
        self.RR_motor.setSelectedSensorPosition(0, 0, 0)

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
        
        SmartDashboard.putNumber("Left Speed", leftPercentage * 4.06)
        SmartDashboard.putNumber("Right Speed", rightPercentage * 4.06)

    def tankDriveVolts(self, leftVolts, rightVolts):
        self.tankDrive(leftVolts / 12, rightVolts / 12)

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

    def getWheelSpeeds(self):
        speeds = DifferentialDriveWheelSpeeds(
            self.getLeftEncoderSpeed(),
            self.getRightEncoderSpeed()
        )
        return speeds

    def getLeftEncoderSpeed(self):
        return self.LF_motor.getSelectedSensorVelocity() * constants.kDrivetrainEncoderDistancePerPulse * 10

    def getRightEncoderSpeed(self):
        return self.RF_motor.getSelectedSensorVelocity() * constants.kDrivetrainEncoderDistancePerPulse * 10

    def getLeftEncoderDistance(self):
        return self.LF_motor.getSelectedSensorPosition() * constants.kDrivetrainEncoderDistancePerPulse

    def getRightEncoderDistance(self):
        return self.RF_motor.getSelectedSensorPosition() * constants.kDrivetrainEncoderDistancePerPulse
