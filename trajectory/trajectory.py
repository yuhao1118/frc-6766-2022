from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.controller import SimpleMotorFeedforwardMeters
import os

import constants
from lib.utils.trajectory import getTrajectoryFromFile


class Trajectories:
    def __init__(self):
        super().__init__()

        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.kDriveKinematics,
            maxVoltage=10,  # 10 volts max.
        )

        # configure forward and reverse trajectories
        forwardConfig = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared
        )

        forwardConfig.setKinematics(constants.kDriveKinematics)
        forwardConfig.addConstraint(autoVoltageConstraint)
        forwardConfig.setReversed(False)

        reverseConfig = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared
        )
        reverseConfig.setKinematics(constants.kDriveKinematics)
        reverseConfig.addConstraint(autoVoltageConstraint)
        reverseConfig.setReversed(True)

        self.trajectoryForward = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            forwardConfig
        )

        self.trajectoryBackward = TrajectoryGenerator.generateTrajectory(
            Pose2d(3, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(0, 0, Rotation2d(0)),
            reverseConfig
        )



class Trajectory:
    ForwardTest = Trajectories().trajectoryForward
    BackwardTest = Trajectories().trajectoryBackward
    Auto11 = getTrajectoryFromFile("Auto1-1.wpilib.json", constants.kTrajectoryDirectory)
    Auto12 = getTrajectoryFromFile("Auto1-2.wpilib.json", constants.kTrajectoryDirectory)
    Auto2 = getTrajectoryFromFile("Auto2.wpilib.json", constants.kTrajectoryDirectory)
