from commands1 import WaitCommand
from commands2 import SubsystemBase, RamseteCommand, InstantCommand, WaitCommand

from wpilib import SerialPort, SmartDashboard, Field2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters

import constants
import ctre
from sensor.wit_imu import WitIMU


class Drivetrain(SubsystemBase):

    def __init__(self):

        super().__init__()
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
        self.setOpenloopRamp(0.5)
        self.resetEncoder()

        self.gyro = WitIMU(SerialPort.Port.kUSB)
        self.gyro.calibrate()
        
        self.odometry = DifferentialDriveOdometry(self.gyro.getRotation2d())

        self.leftPIDController = PIDController(constants.kPDriveVel, 0, 0)
        self.rightPIDController = PIDController(constants.kPDriveVel, 0, 0)

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

    def tankDriveVolts(self, leftVolts, rightVolts):
        self.LF_motor.set(ctre.ControlMode.PercentOutput, leftVolts / 12)
        self.RF_motor.set(ctre.ControlMode.PercentOutput, rightVolts / 12)

    def arcadeDrive(self, throttle, turn):
        if abs(throttle) < 0.05:
            throttle = 0
        
        if abs(turn) < 0.05:
            turn = 0
            
        self.LF_motor.set(ctre.ControlMode.PercentOutput, throttle + turn)
        self.RF_motor.set(ctre.ControlMode.PercentOutput, throttle - turn)

    def zeroHeading(self):
        self.gyro.reset()

    def setMaxOutput(self, maxOutput):
        self.LF_motor.configPeakOutputForward(maxOutput, 20)
        self.LR_motor.configPeakOutputForward(maxOutput, 20)
        self.RF_motor.configPeakOutputForward(maxOutput, 20)
        self.RR_motor.configPeakOutputForward(maxOutput, 20)

    def setOpenloopRamp(self, rampRate):
        self.LF_motor.configOpenloopRamp(rampRate, 20)
        self.LR_motor.configOpenloopRamp(rampRate, 20)
        self.RF_motor.configOpenloopRamp(rampRate, 20)
        self.RR_motor.configOpenloopRamp(rampRate, 20)

    ############## Getter functions ##############

    def getTrajetoryCommand(self, trajectory, shouldInitPose=True):
        def before():
            self.setOpenloopRamp(0)
            if shouldInitPose:
                self.resetOdometry(trajectory.initialPose())

        def after():
            self.setOpenloopRamp(0.5)
            self.tankDriveVolts(0.0, 0.0)

        beforeCommand = InstantCommand(before)
        afterCommand = InstantCommand(after)
        ramseteCommand = RamseteCommand(
            trajectory,
            self.getPose,
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.kDriveKinematics,
            self.getWheelSpeeds,
            self.leftPIDController,
            self.rightPIDController,
            self.tankDriveVolts,
            [self],
        )

        return beforeCommand.andThen(ramseteCommand.andThen(afterCommand))

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
