from commands2 import SequentialCommandGroup, ParallelRaceGroup
from commands.autos.autoconvey import AutoConveyCommandGroup
from commands.conveyor.conveyor import ConveyorCommand
from commands.intake.pneumatic import PneumaticCommand
from commands.autos.autoshoot import AutoShoot, PresetShoot

from trajectory.trajectory import Trajectories


def PathingAndIntakeCommandGroup(robotContainer, trajectory):
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
        AutoConveyCommandGroup(robotContainer),
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
    trajectory11 = Trajectories.Auto11        # 取2球路径
    trajectory12 = Trajectories.Auto12        # 人类玩家站位路径

    return SequentialCommandGroup(
        PresetShoot(
            robotContainer, timeout=1.5, output=60.64, angle=0.0),
        PathingAndIntakeCommandGroup(robotContainer, trajectory11),
        ConveyorCommand(robotContainer, -0.2).withTimeout(0.20),
        AutoShoot(robotContainer),
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
    trajectory = Trajectories.Auto2

    return SequentialCommandGroup(
        PresetShoot(
            robotContainer, timeout=1.5, output=19.3, angle=0.0),
        PathingAndIntakeCommandGroup(robotContainer, trajectory),
        ConveyorCommand(robotContainer, -0.2).withTimeout(0.20),
        AutoShoot(robotContainer),
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
    trajectory1 = Trajectories.Auto31
    trajectory2 = Trajectories.Auto32

    return SequentialCommandGroup(
        PathingAndIntakeCommandGroup(robotContainer, trajectory1),
        ConveyorCommand(robotContainer, -0.2).withTimeout(0.20),
        AutoShoot(robotContainer),
        PathingAndIntakeCommandGroup(robotContainer, trajectory2),
        AutoConveyCommandGroup(robotContainer, reverse=True),
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
