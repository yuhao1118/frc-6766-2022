"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math
from ctre import TalonFXInvertType

# Global
kNominalVoltage = 12.0


# Joystick
kDriverControllerPort = 0


# Motor controller port
kLeftMotor1Port = 6
kLeftMotor2Port = 7
kRightMotor1Port = 8
kRightMotor2Port = 9

kLeftClimbMotorPort = 13
kRightClimbMotorPort = 14

kShooter = 10


# Motor Rotation
kLeftMotorRotate = TalonFXInvertType.Clockwise
kRightMotorRotate = TalonFXInvertType.CounterClockwise

kLeftClimbMotorRotate = TalonFXInvertType.CounterClockwise
kRightClimbMotorRotate = TalonFXInvertType.Clockwise

kShooterRotate = TalonFXInvertType.Clockwise

# Motor to wheel ratio
kEncoderCPR = 2048

kDrivetrainGearRatio = 7
kClimbGearRatio = 15
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

# Drivetrain kinematics
kTrackWidthMeters = 0.585
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)
kOpenloopRampRate = 0.5


# Drivetrain Forward-backward constants
ksVolts = 0.56729
kvVoltSecondsPerMeter = 2.3548
kaVoltSecondsSquaredPerMeter = 0

kP = 2.8
kI = 0.0
kD = 0.0


# Shooter Forward-backward constants
ksVoltsShooter = 0.56601
kvVoltSecondsPerMeterShooter = 0.33631
kaVoltSecondsSquaredPerMeterShooter = 0

shooterSpeedHigh = {
    '0cm': 19.3,
    '35cm': 20,
    '75cm': 21.2,
    '100cm': 23,
    '150cm': 25.5
}

shooterSpeedLow = {
    '0m': 11.3
}

# Max v,a when performing auto-path planning
kMaxSpeedMetersPerSecond = 0.7
kMaxAccelerationMetersPerSecondSquared = 0.4


# Ramsete constants for trajectory following
# Usually not changed
kRamseteB = 2
kRamseteZeta = 0.5
