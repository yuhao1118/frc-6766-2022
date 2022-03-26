"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math
from ctre import TalonFXInvertType

# Joystick
kDriverControllerPort = 0


# Motor controller port
kLeftMotor1Port = 6
kLeftMotor2Port = 7
kRightMotor1Port = 8
kRightMotor2Port = 9

kLeftClimbMotorPort = 13
kRightClimbMotorPort = 14


# Motor Rotation
kLeftMotorRotate = TalonFXInvertType.Clockwise
kRightMotorRotate = TalonFXInvertType.CounterClockwise

kLeftClimbMotorRotate = TalonFXInvertType.CounterClockwise
kRightClimbMotorRotate = TalonFXInvertType.OpposeMaster


# Motor to wheel ratio
kEncoderCPR = 2048

kDrivetrainGearRatio = 7
kClimbGearRatio = 15

kDrivetrainWheelDiameterMeters = 4 * 0.0254
kClimbWheelDiameterMeters = 0.5 * 0.0254

kDriveTrainEncoderDistancePerPulse = (
    kDrivetrainWheelDiameterMeters * math.pi) / (kEncoderCPR * kDrivetrainGearRatio)
kClimbEncoderDistancePerPulse = (
    kClimbWheelDiameterMeters * math.pi) / (kEncoderCPR * kClimbGearRatio)


# Drivetrain kinematics
kTrackWidthMeters = 0.585
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)
kOpenloopRampRate = 0.5

# Forward-backward constants
ksVolts = 0.60019
kvVoltSecondsPerMeter = 2.54
kaVoltSecondsSquaredPerMeter = 0

kP = 0.01
kI = 0.0
kD = 0.0

kNominalVoltage = 12.0

# Ramsete constants for trajectory following
# Usually not changed
kMaxSpeedMetersPerSecond = 3
kMaxAccelerationMetersPerSecondSquared = 3

kRamseteB = 2
kRamseteZeta = 0.5
