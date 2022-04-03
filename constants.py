"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes. All physical quantities are in SI units.
PS: angle is in degrees, not radians.

代码中在多个地方使用的常量值的存放文件. 在这个文件里可以快速, 全局的改变常量的值, 而无需深入理解代码算法, 
达到快速调试的目的. 所有物理量的单位应默认使用国际标准单位制. 
注: 角使用角度制, 而不是弧度制.
"""

from wpimath.kinematics import DifferentialDriveKinematics
from wpilib import RobotBase, RuntimeType
from ctre import TalonFXInvertType

import math
import os

# Limelight address, recommend using static IP address
# 在这里设置limelight的ip地址, 建议使用静态ip地址
kLimelightIp = "10.67.66.30"

# Directory of generated trajectories 
# Pathplanner工具生成轨迹的目录
kTrajectoryDirectory = "/home/lvuser/py/deploy/pathplanner/generatedJSON/" if (RobotBase.getRuntimeType(
) == RuntimeType.kRoboRIO or RobotBase.getRuntimeType() == RuntimeType.kRoboRIO2) else os.getcwd() + "/deploy/pathplanner/generatedJSON/"


kNominalVoltage = 12.0              # 电机峰值电压


# Joystick
# 手柄端口
kDriverControllerPort = 0
kSiderControllerPort = 1


# Intake Solenoid
# Intake继电器端口
kSolenoidLeft = 0
kSolenoidRight = 1


# Motor controller port
# 电机CAN ID
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


# Motor rotation direction
# 电机旋转方向

# 底盘(主)电机旋转方向
kLeftMotorRotate = TalonFXInvertType.Clockwise
kRightMotorRotate = TalonFXInvertType.CounterClockwise

# 爬升电机旋转方向
kLeftClimbMotorRotate = TalonFXInvertType.CounterClockwise
kRightClimbMotorRotate = TalonFXInvertType.Clockwise

# 爬升摇臂电机旋转方向
kLeftCLimbArmMotorRotate = TalonFXInvertType.Clockwise
kRightCLimbArmMotorRotate = TalonFXInvertType.OpposeMaster

# 射球电机旋转方向
kShooterRotate = TalonFXInvertType.Clockwise

# 传送带电机旋转方向
kConveyorRotate = TalonFXInvertType.CounterClockwise

# Intake电机旋转方向
kIntakeRotate = TalonFXInvertType.CounterClockwise


# Motor to wheel ratio
# 电机-输出轮比例

kEncoderCPR = 2048                  # 编码器脉冲数(一圈)

kDrivetrainGearRatio = 7            # 底盘点击减速比
kClimbGearRatio = 15                # 爬升电机减速比
kClimbArmGearRatio = 130.5          # 爬升摇臂电机减速比
kShooterGearRatio = 1               # 射球电机减速比

kDrivetrainWheelDiameterMeters = 4 * 0.0254         # 底盘轮周长(m)
kShooterWheelDiameterMeters = 4 * 0.0254            # 射球轮周长(m)
kClimbWheelDiameterMeters = 0.5 * 0.0254            # 爬升轮周长(m)

kDrivetrainEncoderDistancePerPulse = (
    kDrivetrainWheelDiameterMeters * math.pi) / (kEncoderCPR * kDrivetrainGearRatio)        # 底盘轮脉冲距离(m): 一个脉冲相当于轮子走多少距离
kClimbEncoderDistancePerPulse = (
    kClimbWheelDiameterMeters * math.pi) / (kEncoderCPR * kClimbGearRatio)                  # 爬升轮脉冲距离(m): 一个脉冲相当于轮子走多少距离
kShooterEncoderDistancePerPulse = (
    kShooterWheelDiameterMeters * math.pi) / (kEncoderCPR * kShooterGearRatio)              # 射球轮脉冲距离(m): 一个脉冲相当于轮子走多少距离
kClimbArmEncoderDegreesPerPulse = 360 / (kEncoderCPR * kClimbArmGearRatio)                  # 爬升摇臂脉冲角度(°): 一个脉冲相当于摇臂转多少角度


# Climber motor-safety constants
# 爬升电机安全限制

# Once the kClimbMotorThresholdCurrent is reached for kClimbMotorThresholdDuration seconds,
# the motor will limit the current to kClimbMotorCurrentLimit.
# 当爬升电机的电流达到<kClimbMotorThresholdCurrent>，并持续<kClimbMotorThresholdDuration>秒时，
# 将电机电流维持在<kClimbMotorCurrentLimit>
kClimbMotorCurrentLimit = 40
kClimbMotorThresholdCurrent = 60
kClimbMotorThresholdDuration = 0.8

# Climber motor soft limits
# 爬升电机软限位 (以伸缩杆完全收紧时为基准)
kClimbMotorSoftLimitForward = 0.0 / kClimbEncoderDistancePerPulse           # 前向限位: 0m
kClimbMotorSoftLimitReverse = -0.31 / kClimbEncoderDistancePerPulse         # 后向限位: -0.31m

# Climb arm motor soft limits
# 爬升摇臂电机软限位 (以摇臂竖直时为基准)
kClimbArmMotorSoftLimitForward = 30 / kClimbArmEncoderDegreesPerPulse       # 前向限位: 30°
kClimbArmMotorSoftLimitReverse = -30 / kClimbArmEncoderDegreesPerPulse      # 后向限位: -30°

# Drivetrain kinematics
# 底盘运动学
kTrackWidthMeters = 0.585                                                               # 水平轮距(m)
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)                       # 底盘运动学常量

# Let the RamseteController handle the acceleration
# 自动阶段底盘加速时间为0s, 因为生成的路径已经做过加速度限制了
kOpenloopRampRateAuto = 0.0

# 手动阶段加速时间为0.5s
kOpenloopRampRateTeleop = 0.5

kDeadband = 0.07                                        # 手柄死区，在此正负区间内的手柄输入值会被忽略

kDrivetrainMaxOutput = 0.8                              # 底盘最大输出
kDrivetrainTurnSensitive = 0.65                         # 底盘转向灵敏度

# Drivetrain Forward-backward constants
# 底盘前向控制常量, 由Sysid工具计算得到
ksVolts = 0.56729                                       
kvVoltSecondsPerMeter = 2.3548
kaVoltSecondsSquaredPerMeter = 0

# Drivetrain PID
# 底盘PID
kP = 1.0
kI = 0.0
kD = 0.0

# Max v,a when performing auto-path planning
# 自动路径规划时的最大速度和加速度
kMaxSpeedMetersPerSecond = 3.0
kMaxAccelerationMetersPerSecondSquared = 1.5


# Ramsete constants for trajectory following, usually not changed
# Ramsete轨迹跟踪常量, 通常无需更改
kRamseteB = 2
kRamseteZeta = 0.5


# Shooter forward constants
# 射球前向控制常量, 由Sysid工具计算得到
ksVoltsShooter = 0.56601
kvVoltSecondsPerMeterShooter = 0.33631
kaVoltSecondsSquaredPerMeterShooter = 0

# Shooter PID
# 射球PID
kPShooter = 0.1
kIShooter = 0.0
kDShooter = 0.0


# Vision distance measurement constants, ignore them at the moment
# 视觉测距测量 (暂时不用)
kVisionTargetHeight = 2.58                  # (meters) Height of the target off the ground
kVisionCameraHeight = 0.63                  # (meters) Height of the camera off the ground
kVisionCameraPitch = math.radians(60)       # (radians) Pitch of the camera

# Vision turn PID constants
# 视觉自瞄PID
kPVisionTurn = 0.01
kIVisionTurn = 0.0
kDVisionTurn = 0.0

# Vision forward PID constants, ignore them at the moment
# 视觉距离调整PID (暂时不用)
kPVisionForward = 0.01
kIVisionForward = 0.0
kDVisionForward = 0.0
