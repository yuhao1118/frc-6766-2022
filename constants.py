"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math
from ctre import TalonFXInvertType

# ID for the driver's joystick.
kDriverControllerPort = 0

# The PWM IDs for the drivetrain motor controllers.
kLeftMotor1Port = 6
kLeftMotor2Port = 7
kRightMotor1Port = 8
kRightMotor2Port = 9

# The rotate direction of the drivetrain motors.
kLeftMotorRotate = TalonFXInvertType.Clockwise
kRightMotorRotate = TalonFXInvertType.CounterClockwise

# In meters, distance between wheels on each side of robot.
kTrackWidthMeters = 0.585
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

# Encoder counts per revolution/rotation.
kEncoderCPR = 2048
kGearRatio = 7
kWheelDiameterMeters = 4 * 0.0254

# The following works assuming the encoders are directly mounted to the wheel shafts.
kEncoderDistancePerPulse = (
    kWheelDiameterMeters * math.pi) / (kEncoderCPR * kGearRatio)

# NOTE: Please do NOT use these values on your robot. Rather, characterize your
# drivetrain using the FRC Characterization tool. These are for demo purposes
# only!
ksVolts = 0.22
kvVoltSecondsPerMeter = 1.98
kaVoltSecondsSquaredPerMeter = 0.2

# The P gain for our turn controllers.
kPDriveVel = 2.95

# The max velocity and acceleration for our autonomous.
kMaxSpeedMetersPerSecond = 3
kMaxAccelerationMetersPerSecondSquared = 3

# Baseline values for a RAMSETE follower in units of meters
# and seconds. These are recommended, but may be changes if wished.
kRamseteB = 2
kRamseteZeta = 0.7

# The number of motors on the robot.
kDrivetrainMotorCount = 4
