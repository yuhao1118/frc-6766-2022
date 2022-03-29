from commands2 import SequentialCommandGroup, ParallelRaceGroup, ParallelCommandGroup, WaitCommand
from wpimath.trajectory import Trajectory, TrajectoryUtil
from wpimath.geometry import Pose2d, Rotation2d
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup
from commands import shootercommand

from trajectory.trajectory import Trajectory


def pathingAndIntakeCommandGroup(robotContainer, trajectory):
    return ParallelRaceGroup(
        robotContainer.robotDrive.getTrajectoryCommand(trajectory),
        IntakeConveyCommandGroup(robotContainer)
    )


def Auto1CommandGroup(robotContainer):
    trajectory11 = Trajectory.Auto11
    trajectory12 = Trajectory.Auto12

    # Total time: 12.41 seconds
    # 3 balls shooted at around t=8.66s
    return SequentialCommandGroup(
        # 2.0s
        AutoShootCommandGroup(robotContainer),
        # 4.86s
        pathingAndIntakeCommandGroup(
            robotContainer, trajectory),
        # 1s
        WaitCommand(1.0),
        # 2.0s
        AutoShootCommandGroup(robotContainer),
        # 2.55s
        robotContainer.robotDrive.getTrajectoryCommand(trajectory12),
    )


def TestCommandGroup(robotContainer, trajectory):
    return robotContainer.robotDrive.getTrajectoryCommand(trajectory)
