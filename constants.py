"""
项目中在被多次使用的常量存放文件。 在这个文件里可以快速的改变全局常量而无需深入理解代码算法，从而
达到快速调试的目的。如无特殊说明，所有物理量的单位应默认使用国际标准单位制。
"""

from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.geometry import Transform2d, Rotation2d, Translation2d
from wpilib import RobotBase, RuntimeType
from ctre import TalonFXInvertType
import math
import os

tuningMode = True
kTrajectoryDirectory = "/home/lvuser/py/deploy/pathplanner/generatedJSON/" \
    if RobotBase.getRuntimeType() in [RuntimeType.kRoboRIO, RuntimeType.kRoboRIO2] \
    else os.getcwd() + "/deploy/pathplanner/generatedJSON/"
kNominalVoltage = 12.0              # 电机峰值电压

# 控制端口
## Limelight静态IP
kLimelightIp = "10.67.66.30"
## 手柄端口
kDriverControllerPort = 0
kSiderControllerPort = 1
## 电磁阀端口
kSolenoidLeft = 0
kSolenoidRight = 1
## 电机CAN ID
kLeftMotor1Port = 6
kLeftMotor2Port = 7
kRightMotor1Port = 8
kRightMotor2Port = 9
kLeftClimbMotorPort = 14
kRightClimbMotorPort = 13
kLeftClimbArmMotorPort = 15
kRightClimbArmMotorPort = 16
kShooteMotorPort = 10
kHoodMotorPort = 17
kConveyorMotorPort = 11
kIntakeMotorPort = 12


# 电机旋转方向
kLeftMotorRotate = TalonFXInvertType.Clockwise                  # 底盘左电机旋转方向
kRightMotorRotate = TalonFXInvertType.CounterClockwise          # 底盘右电机旋转方向
kLeftClimbMotorRotate = TalonFXInvertType.Clockwise             # 左爬升电机旋转方向
kRightClimbMotorRotate = TalonFXInvertType.CounterClockwise     # 右爬升电机旋转方向
kLeftCLimbArmMotorRotate = TalonFXInvertType.Clockwise          # 左爬升摇臂电机旋转方向
kRightCLimbArmMotorRotate = TalonFXInvertType.OpposeMaster      # 又爬升摇臂电机旋转方向
kShooterRotate = TalonFXInvertType.Clockwise                    # 射球电机旋转方向
kConveyorRotate = TalonFXInvertType.CounterClockwise            # 传送带电机旋转方向
kIntakeRotate = TalonFXInvertType.CounterClockwise              # Intake电机旋转方向
kHoodRotate = TalonFXInvertType.Clockwise                       # 射球罩电机旋转方向


# 电机单位换算
kEncoderCPR = 2048                                              # Falcon500编码器旋转一圈脉冲数
kDrivetrainGearRatio = 7                                        # 底盘点击减速比
kClimbArmGearRatio = 130.5                                      # 爬升摇臂电机减速比
kShooterGearRatio = 1                                           # 射球电机减速比
kHoodGearRatio = 480 / 18 * 48 / 24                             # 射球罩电机减速比
kDrivetrainWheelDiameterMeters = 4 * 0.0254                     # (m) 底盘轮周长
## 底盘轮脉冲距离(m): 一个脉冲相当于轮子走多少距离
kDrivetrainEncoderDistancePerPulse = (kDrivetrainWheelDiameterMeters * math.pi) / (kEncoderCPR * kDrivetrainGearRatio)
## 射球轮脉冲距离(m): 一个脉冲相当于轮子走多少距离
kShooterEncoderRotatePerPulse = 1 / (kEncoderCPR * kShooterGearRatio)
## 爬升摇臂脉冲角度(°): 一个脉冲相当于摇臂转多少角度
kClimbArmEncoderDegreesPerPulse = 360 / (kEncoderCPR * kClimbArmGearRatio)
## 射球罩脉冲角度(°): 一个脉冲相当于罩转多少角度
kHoodEncoderDegreesPerPulse = 360 / (kEncoderCPR * kHoodGearRatio)


# 电机安全限制
## 当爬升电机的电流达到<kClimbMotorThresholdCurrent>，并持续<kClimbMotorThresholdDuration>秒时，
## 将电机电流维持在<kClimbMotorCurrentLimit>
kClimbMotorCurrentLimit = 40
kClimbMotorThresholdCurrent = 60
kClimbMotorThresholdDuration = 0.8
## 爬升电机软限位 (以伸缩杆完全收紧时为基准)
kClimbMotorSoftLimitForward = 0                                     # (脉冲) 前向限位
kClimbMotorSoftLimitReverse = -355000                               # (脉冲) 后向限位
## 射球罩电机软限位 (以射球罩完全收起时为基准)
kHoodMotorSoftLimitForward = 20 / kHoodEncoderDegreesPerPulse       # (°) 前向限位


# 底盘运动学
kTrackWidthMeters = 0.585                                           # (m) 水平轮距
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)   # 底盘运动学常量
kCurvatureThreshold = 0.1                                           # 曲率阈值，超过此值则认为是曲率运动
kDrivetrainTurnSensitive = 0.3                                      # 转向灵敏度
kDrivetrainMaxOutput = 0.8                                          # 底盘最大输出
kDrivetrainMotorCount = 4                                           # 底盘电机数量
## 底盘前向控制常量, 由Sysid工具计算得到
ksVolts = 0.6191
kvVoltSecondsPerMeter = 2.3148
kaVoltSecondsSquaredPerMeter = 0.2779
## 自动路径规划时的最大速度和加速度
kMaxSpeedMetersPerSecond = 3.0
kMaxAccelerationMetersPerSecondSquared = 1.5
## Ramsete轨迹跟踪常量, 通常无需更改
kRamseteB = 2
kRamseteZeta = 0.7


# 场地常量
kFieldLengthMeters = 16.46                                                   # (m) 赛场长度
kFieldWidthMeters = 8.23                                                     # (m) 赛场宽度
kHubHeightLower = 2.58                                                       # (m) HUB下沿距地面高度
kHubHeightHigher = 2.64                                                      # (m) HUB上沿距地面高度
kHubRadiusMeter = 0.678                                                      # (m) HUB半径
kHubCenter = Translation2d(kFieldLengthMeters / 2, kFieldWidthMeters / 2)    # (m) HUB坐标


# 相机常量
kCameraHeight = 0.63                                                    # (m) 摄像头距地面高度
kCameraPitch = Rotation2d.fromDegrees(52.0)                             # (°) 摄像头与水平夹角
kCameraOffset = Transform2d(Translation2d(0.3, 0.0), Rotation2d())      # (m) 摄像头与底盘中心偏移
kCameraFovHorizontal = Rotation2d.fromDegrees(59.6)                     # (°) 摄像头水平视角
kCameraFovVertical = Rotation2d.fromDegrees(49.7)                       # (°) 摄像头垂直视角


# 视觉常量
kVisionFilterTime = 0.1
kVisionFilterPeriod = 0.02
kVisionLatencyMs = 20
kVisionNominalFramerate = 22
kVisionWidthPixel = 960
kVisionHeightPixel = 720
kVisionCrosshairX = 0.0
kVisionCrosshairY = 0.0
