import math
from commands2 import SubsystemBase, RamseteCommand, InstantCommand, SequentialCommandGroup

from wpilib import SerialPort, SmartDashboard, Field2d, RobotBase, Timer
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters
# from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
import ctre

import constants
from lib.sensors.wit_imu import WitIMU
from lib.utils.tunablenumber import TunableNumber


class Drivetrain(SubsystemBase):

    def __init__(self, visionControl):
        super().__init__()
        self.feedforwardController = SimpleMotorFeedforwardMeters(
            constants.ksVolts,
            constants.kvVoltSecondsPerMeter,
            constants.kaVoltSecondsSquaredPerMeter,
        )

        self.field2d = Field2d()    # Display field image in dashboard
        self.field2dVision = Field2d()
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
        # self.visionOdometry = DifferentialDrivePoseEstimator(self.gyro.getRotation2d(),
        #                                                      Pose2d(),
        #                                                      (0.05, 0.05, math.radians(
        #                                                          5), 0.01, 0.01),
        #                                                      (0.02, 0.02,
        #                                                       math.radians(1)),
        #                                                      (0.5, 0.5, math.radians(30)))
        self.visionOdometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d())

        self.kP = TunableNumber("Drivetrain/kP", 1.0)
        self.kI = TunableNumber("Drivetrain/kI", 0.0)
        self.kD = TunableNumber("Drivetrain/kD", 0.0)

        self.leftPIDController = PIDController(
            self.kP.getDefault(), self.kI.getDefault(), self.kD.getDefault())
        self.rightPIDController = PIDController(
            self.kP.getDefault(), self.kI.getDefault(), self.kD.getDefault())

        self.visionControl = visionControl

    def log(self):
        SmartDashboard.putData("Drivetrain", self)
        SmartDashboard.putData("Field2d", self.field2d)
        SmartDashboard.putData("Field2dVision", self.field2dVision)
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

    def _updateOdometry(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.getLeftEncoderDistance(),
            self.getRightEncoderDistance()
        )

        # self.visionOdometry.update(
        #     self.gyro.getRotation2d(),
        #     self.getWheelSpeeds(),
        #     self.getLeftEncoderDistance(),
        #     self.getRightEncoderDistance())

        self.visionOdometry.update(
            self.gyro.getRotation2d(),
            self.getLeftEncoderDistance(),
            self.getRightEncoderDistance()
        )

        if self.visionControl.hasTargets():
            # self.visionOdometry.addVisionMeasurement(
            #     self.visionControl.getRobotPose(
            #         self.getPose(fromVisionOdometry=True).rotation()),
            #     Timer.getFPGATimestamp() - self.visionControl.getLatency()
            # )
            poseFromVision = self.visionControl.getRobotPose(
                self.getPose(fromVisionOdometry=True).rotation())

            self.resetOdometry(poseFromVision, visionOdometryOnly=True)

        self.field2d.setRobotPose(self.getPose())
        self.field2dVision.setRobotPose(self.getPose(fromVisionOdometry=True))

    def periodic(self):
        if self.kP.hasChanged():
            self.leftPIDController.setP(self.kP.get())
            self.rightPIDController.setP(self.kP.get())

        if self.kI.hasChanged():
            self.leftPIDController.setI(self.kI.get())
            self.rightPIDController.setI(self.kI.get())

        if self.kD.hasChanged():
            self.leftPIDController.setD(self.kD.get())
            self.rightPIDController.setD(self.kD.get())

        self._updateOdometry()

        if RobotBase.isSimulation():
            self.log()

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

    def resetOdometry(self, pose, visionOdometryOnly=False):
        self.resetEncoder()

        if visionOdometryOnly:
            self.visionOdometry.resetPosition(pose, self.gyro.getRotation2d())
            self.odometry.resetPosition(
                self.getPose(), self.gyro.getRotation2d())
        else:
            self.visionOdometry.resetPosition(pose, self.gyro.getRotation2d())
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

    def getPose(self, fromVisionOdometry=False):
        # return the estimated pose of the robot
        if fromVisionOdometry:
            # return self.visionOdometry.getEstimatedPosition()
            return self.visionOdometry.getPose()

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
