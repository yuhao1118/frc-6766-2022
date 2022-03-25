# wpimath is a module that helps us with poses, generating trajectories, etc.
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.controller import SimpleMotorFeedforwardMeters

import constants


class PathTrajectory:
    def __init__(self):
        # super init (initializes sequentialcommandgroup)
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

        # self.trajectoryBlue1p1 = TrajectoryGenerator.generateTrajectory(
        #     Pose2d(6.95, 4.64, Rotation2d(157.09)),
        #     [Translation2d(6.2, 5.16)],
        #     Pose2d(5.53, 5.64, Rotation2d(144.25)),
        #     reverseConfig
        # )

        # self.trajectoryBlue1p2 = TrajectoryGenerator.generateTrajectory(
        #     Pose2d(5.53, 5.64, Rotation2d(144.25)),
        #     [Translation2d(6.2, 5.16)],
        #     Pose2d(6.95, 4.64, Rotation2d(157.09)),
        #     forwardConfig
        # )
