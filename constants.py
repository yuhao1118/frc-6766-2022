"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics
from wpilib import RobotBase, RuntimeType
from ctre import TalonFXInvertType

import math
import os

# Generate paths directory
kTrajectoryDirectory = "/home/lvuser/py/deploy/pathplanner/generatedJSON/" if (RobotBase.getRuntimeType(
) == RuntimeType.kRoboRIO or RobotBase.getRuntimeType() == RuntimeType.kRoboRIO2) else os.getcwd() + "/deploy/pathplanner/generatedJSON/"

# Global
kNominalVoltage = 12.0


# Joystick
kDriverControllerPort = 0
kSiderControllerPort = 1


# Intake Solenoid
kSolenoidLeft = 0
kSolenoidRight = 1


# Motor controller port
kLeftMotor1Port = 6
kLeftMotor2Port = 7
kRightMotor1Port = 8
kRightMotor2Port = 9

kLeftClimbMotorPort = 13
kRightClimbMotorPort = 14

kLeftClimbArmMotorPort = 15
kRightClimbArmMotorPort = 16

kShooter = 10

kConveyorPort = 11
kIntakePort = 12


# Motor Rotation
kLeftMotorRotate = TalonFXInvertType.Clockwise
kRightMotorRotate = TalonFXInvertType.CounterClockwise

kLeftClimbMotorRotate = TalonFXInvertType.CounterClockwise
kRightClimbMotorRotate = TalonFXInvertType.Clockwise

kLeftCLimbArmMotorRotate = TalonFXInvertType.CounterClockwise
kRightCLimbArmMotorRotate = TalonFXInvertType.OpposeMaster

kShooterRotate = TalonFXInvertType.Clockwise

kConveyorRotate = TalonFXInvertType.CounterClockwise
kIntakeRotate = TalonFXInvertType.CounterClockwise


# Motor to wheel ratio
kEncoderCPR = 2048

kDrivetrainGearRatio = 7
kClimbGearRatio = 15
kClimbArmGearRatio = 130.5
kShooterGearRatio = 1

kDrivetrainWheelDiameterMeters = 4 * 0.0254
kShooterWheelDiameterMeters = 4 * 0.0254
kClimbWheelDiameterMeters = 0.5 * 0.0254

kDriveTrainEncoderDistancePerPulse = (
    kDrivetrainWheelDiameterMeters * math.pi) / (kEncoderCPR * kDrivetrainGearRatio)
kClimbEncoderDistancePerPulse = (
    kClimbWheelDiameterMeters * math.pi) / (kEncoderCPR * kClimbGearRatio)
kShooterEncoderDistancePerPulse = (
    kShooterWheelDiameterMeters * math.pi) / (kEncoderCPR * kShooterGearRatio)
kClimbArmEncoderDegreesPerPulse = 360 / (kEncoderCPR * kClimbArmGearRation)


# Climber motor-safety constants

# Once the kClimbMotorThresholdCurrent is reached for kClimbMotorThresholdDuration seconds,
# the motor will limit the current to kClimbMotorCurrentLimit.
kClimbMotorCurrentLimit = 40
kClimbMotorThresholdCurrent = 50
kClimbMotorThresholdDuration = 0.5

kClimbMotorSoftLimitForward = 10 / kClimbEncoderDistancePerPulse
kClimbMotorSoftLimitReverse = -26.9 / kClimbEncoderDistancePerPulse

kClimbArmMotorSoftLimitForward = 60 / kClimbArmEncoderDegreesPerPulse
kClimbArmMotorSoftLimitReverse = -10 / kClimbArmEncoderDegreesPerPulse

kPClimbArm = 0.001
kIClimbArm = 0.0
kDClimbArm = 0.0

# Drivetrain kinematics
kTrackWidthMeters = 0.585
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)
kOpenloopRampRateTeleop = 0.25
kOpenloopRampRateAuto = 0.0     # Let the RamseteController handle the acceleration
kDeadband = 0.07

# Drivetrain Forward-backward constants
kDrivetrainMaxOutput = 0.85

ksVolts = 0.56729
kvVoltSecondsPerMeter = 2.3548
kaVoltSecondsSquaredPerMeter = 0

kP = 1.0
kI = 0.0
kD = 0.0

# Max v,a when performing auto-path planning
kMaxSpeedMetersPerSecond = 3.00
kMaxAccelerationMetersPerSecondSquared = 3.00


# Ramsete constants for trajectory following
# Usually not changed
kRamseteB = 2
kRamseteZeta = 0.5


# Shooter Forward-backward constants
ksVoltsShooter = 0.56601
kvVoltSecondsPerMeterShooter = 0.33631
kaVoltSecondsSquaredPerMeterShooter = 0

kPShooter = 0.1
kIShooter = 0.0
kDShooter = 0.0

shooterSpeedHigh = {
    '0cm': 19.3,
    '35cm': 20,
    '75cm': 21.2,
    '100cm': 23,
    '150cm': 25.5
}

shooterSpeedLow = {
    '0cm': 11.3
}


# Shooter Vision
kVisionTargetHeight = 2.58   # (meters) Height of the target off the ground
kVisionCameraHeight = 0.63    # (meters) Height of the camera off the ground
kVisionCameraPitch = math.radians(60)     # (radians) Pitch of the camera

kPVisionTurn = 0.01
kIVisionTurn = 0.0
kDVisionTurn = 0.0

kPVisionForward = 0.01
kIVisionForward = 0.0
kDVisionForward = 0.0
