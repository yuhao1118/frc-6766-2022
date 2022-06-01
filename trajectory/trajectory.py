from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.controller import SimpleMotorFeedforwardMeters

import constants
from lib.utils.trajectory import getTrajectoryFromFile


class Trajectory:

    @classmethod
    def getConfig(cls, reversed=False):
        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.kDriveKinematics,
            maxVoltage=10,  # 10 volts max.
        )

        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared
        )

        config.setKinematics(constants.kDriveKinematics)
        config.addConstraint(autoVoltageConstraint)
        config.setReversed(reversed)

        return config

    @classmethod
    def getForwardTestTrajectory(cls):
        return TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            cls.getConfig()
        )

    @classmethod
    def getBackwardTestTrajectory(cls):
        return TrajectoryGenerator.generateTrajectory(
            Pose2d(3, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(0, 0, Rotation2d(0)),
            cls.getConfig(reversed=True)
        )


class Trajectories:
    """
    存放路径的常量类
    """
    # 测试用的路径
    # 前进z字形路径
    ForwardTest = Trajectory.getForwardTestTrajectory()
    # 后退z字形路径
    BackwardTest = Trajectory.getBackwardTestTrajectory()

    # 自动阶段用的路径
    Auto11 = getTrajectoryFromFile(
        "Auto1-1.wpilib.json", constants.kTrajectoryDirectory)   # 自动路径1-1
    Auto12 = getTrajectoryFromFile(
        "Auto1-2.wpilib.json", constants.kTrajectoryDirectory)   # 自动路径1-2
    Auto2 = getTrajectoryFromFile(
        "Auto2.wpilib.json", constants.kTrajectoryDirectory)      # 自动路径2
    Auto31 = getTrajectoryFromFile(
        "Auto3-1.wpilib.json", constants.kTrajectoryDirectory)   # 自动路径3-1
    Auto32 = getTrajectoryFromFile(
        "Auto3-2.wpilib.json", constants.kTrajectoryDirectory)   # 自动路径3-2
