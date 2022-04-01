from commands2 import SequentialCommandGroup, ParallelRaceGroup, WaitCommand
from wpimath.trajectory import Trajectory
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup

from trajectory.trajectory import Trajectory

def pathingAndIntakeCommandGroup(robotContainer, trajectory):
    """
    寻路取球并行指令组:
    1. 吸球 + 传球
    2. 沿给定的路径 (trajectory) 运动
    """
    return ParallelRaceGroup(
        robotContainer.robotDrive.getTrajectoryCommand(trajectory),
        IntakeConveyCommandGroup(robotContainer)
    )


def Auto1CommandGroup(robotContainer):
    """
    自动3球+取1球:
    1. 射出1球
    2. 执行<寻路取球并行指令组>, 取2球
    3. 射出2球
    4. 寻路至人类玩家站位
    """
    trajectory11 = Trajectory.Auto11        # 取2球路径
    trajectory12 = Trajectory.Auto12        # 人类玩家站位路径

    return SequentialCommandGroup(
        AutoShootCommandGroup(robotContainer, backBallTime=0.2, timeout=1.5),
        pathingAndIntakeCommandGroup(robotContainer, trajectory11),
        AutoShootCommandGroup(robotContainer),
        # robotContainer.robotDrive.getTrajectoryCommand(trajectory12),
    )


def Auto2CommandGroup(robotContainer):
    """
    自动2球:
    1. 射出1球
    2. 执行<寻路取球并行任务>, 取1球
    3. 射出1球
    """
    trajectory = Trajectory.Auto21 + Trajectory.Auto22

    return SequentialCommandGroup(
        AutoShootCommandGroup(robotContainer, backBallTime=0.2, timeout=1.5),
        pathingAndIntakeCommandGroup(robotContainer, trajectory),
        AutoShootCommandGroup(robotContainer),
    )

def TestCommandGroup(robotContainer, trajectory):
    """
    测试路径:
    1. 沿给定的路径 (trajectory) 运动
    """
    return robotContainer.robotDrive.getTrajectoryCommand(trajectory)
