import math
from commands2 import SubsystemBase, RamseteCommand, InstantCommand, SequentialCommandGroup

from wpilib import SerialPort, SmartDashboard, Field2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters
import ctre

import constants
from lib.enums.pov import POVEnum
from lib.sensors.wit_imu import WitIMU
from lib.utils.math import clamp
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
        self.LF_motor.configOpenloopRamp(seconds, 20)
        self.LR_motor.configOpenloopRamp(seconds, 20)
        self.RF_motor.configOpenloopRamp(seconds, 20)
        self.RR_motor.configOpenloopRamp(seconds, 20)

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

    def tankDriveVolts(self, leftVolts, rightVolts):
        self.tankDrive(leftVolts / 12, rightVolts / 12)

    def tankDriveVelocity(self, leftVel, rightVel):
        leftVolts = self.feedforwardController.calculate(leftVel) + self.leftPIDController.calculate(self.getLeftEncoderSpeed(), leftVel)
        rightVolts = self.feedforwardController.calculate(rightVel) + self.rightPIDController.calculate(self.getRightEncoderSpeed(), rightVel)
        SmartDashboard.putNumber("Left Volts", leftVolts)
        SmartDashboard.putNumber("Right Volts", rightVolts)
        self.tankDriveVolts(leftVolts, rightVolts)

    def arcadeDrive(self, throttle, turn, smoothInputs=True):
        if smoothInputs:
            if abs(throttle) < 0.07:
                throttle = 0
            
            if abs(turn) < 0.07:
                turn = 0

            turn = turn ** 3 * constants.kDrivetrainTurnSensitive
            throttle = throttle ** 3

        leftSpeed = clamp(throttle + turn, -1.0, 1.0)
        rightSpeed = clamp(throttle - turn, -1.0, 1.0)

        self.tankDrive(leftSpeed, rightSpeed)

    def povDrive(self, povButton):
        # Using POV button to adjust the drivetrain

        if povButton == POVEnum.kUp:
            self.arcadeDrive(0.2, 0, smoothInputs=False)
        elif povButton == POVEnum.kDown:
            self.arcadeDrive(-0.2, 0, smoothInputs=False)
        elif povButton == POVEnum.kRight:
            self.arcadeDrive(0, 0.2, smoothInputs=False)
        elif povButton == POVEnum.kLeft:
            self.arcadeDrive(0, -0.2, smoothInputs=False)

    def zeroHeading(self):
        self.gyro.reset()

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

        if shouldInitPose:
            return SequentialCommandGroup(
                InstantCommand(lambda: self.resetOdometry(
                    trajectory.initialPose())),
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
