from commands2 import SequentialCommandGroup, ParallelRaceGroup
from wpimath.trajectory import Trajectory
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup
from commands.pneumaticcommand import PneumaticCommand
from commands.autos.getrangeandaim import GetRangeAndAimCommand

from trajectory.trajectory import Trajectory

def pathingAndIntakeCommandGroup(robotContainer, trajectory):
    """
    输入:
    1. robotContainer: RobotContainer实例
    2. trajectory: 路径

    寻路取球并行指令组:
    1. 吸球 + 传球
    2. 沿给定的路径 (trajectory) 运动
    """
    return ParallelRaceGroup(
        robotContainer.robotDrive.getTrajectoryCommand(trajectory),
        IntakeConveyCommandGroup(robotContainer),
        PneumaticCommand(robotContainer, True)
    )


def Auto1CommandGroup(robotContainer):
    """
    输入:
    1. robotContainer: RobotContainer实例

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
        GetRangeAndAimCommand(robotContainer).withTimeout(0.8),
        AutoShootCommandGroup(robotContainer),
        # robotContainer.robotDrive.getTrajectoryCommand(trajectory12),
    )


def Auto2CommandGroup(robotContainer):
    """
    输入:
    1. robotContainer: RobotContainer实例

    自动2球:
    1. 射出1球
    2. 执行<寻路取球并行任务>, 取1球
    3. 射出1球
    """
    trajectory = Trajectory.Auto2

    return SequentialCommandGroup(
        AutoShootCommandGroup(robotContainer, backBallTime=0.2, timeout=1.5),
        pathingAndIntakeCommandGroup(robotContainer, trajectory),
        AutoShootCommandGroup(robotContainer),
    )

def Auto3CommandGroup(robotContainer):
    """
    输入:
    1. robotContainer: RobotContainer实例

    自动2球:
    1. 执行<寻路取球并行任务>, 取1球
    2. 射出2球
    3. 执行<寻路取球并行任务>, 取对方1球
    4. 放对方球至我方爬升区
    """
    trajectory1 = Trajectory.Auto31
    trajectory2 = Trajectory.Auto32

    return SequentialCommandGroup(
        pathingAndIntakeCommandGroup(robotContainer, trajectory1),
        AutoShootCommandGroup(robotContainer),
        pathingAndIntakeCommandGroup(robotContainer, trajectory2),
        IntakeConveyCommandGroup(robotContainer, reverse=True),
    )

def TestCommandGroup(robotContainer, trajectory):
    """
    输入:
    1. robotContainer: RobotContainer实例
    2. trajectory: 路径
    
    测试路径:
    1. 沿给定的路径 (trajectory) 运动
    """
    return robotContainer.robotDrive.getTrajectoryCommand(trajectory)
