from commands2 import SequentialCommandGroup, ParallelRaceGroup, WaitCommand
from wpimath.trajectory import Trajectory
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup

from trajectory.trajectory import Trajectory

def pathingAndIntakeCommandGroup(robotContainer, trajectory):
    return ParallelRaceGroup(
        robotContainer.robotDrive.getTrajectoryCommand(trajectory),
        IntakeConveyCommandGroup(robotContainer)
    )


def Auto1CommandGroup(robotContainer):
    trajectory11 = Trajectory.Auto11
    trajectory12 = Trajectory.Auto12

    return SequentialCommandGroup(
        # 1.3s
        AutoShootCommandGroup(robotContainer, backBallTime=0.2, timeout=1.5),
        # 4.95s
        pathingAndIntakeCommandGroup(
            robotContainer, trajectory11),
        # 2.0s
        AutoShootCommandGroup(robotContainer),
        # 3.09s
        # robotContainer.robotDrive.getTrajectoryCommand(trajectory12),
    )


def TestCommandGroup(robotContainer, trajectory):
    return robotContainer.robotDrive.getTrajectoryCommand(trajectory)
