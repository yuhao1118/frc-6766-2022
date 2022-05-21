from commands.shoot.flywheel import FlywheelCommand
from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from commands.conveyor.conveyor import ConveyorCommand


def AutoShootCommandGroup(robotContainer, timeout=2.2, backBallTime=0.3, output=None):
    """
    自动射球并行指令组:

    输入:
    1. robotContainer: RobotContainer实例
    2. timeout: 总指令组超时时间, 默认为2.2秒
    3. backBallTime: Conveyor退球时间, 默认为0.3秒
    4. output: 射球速度, 默认为在0cm处射球时所需电机速度

    输出:
    并行指令组:
    1. 射球电机延迟<0.1s>启动, 因为需要时间给Conveyor退球
    2. 顺序指令组:
        2.1. Conveyor退球, 速度-0.2, 时间<backBallTime>
        2.2. Conveyor送球延迟<0.3s>启动, 速度0.3
    """
    return ParallelCommandGroup(
        WaitCommand(0.1).andThen(FlywheelCommand(robotContainer, output=output)),
        SequentialCommandGroup(
            ConveyorCommand(
                robotContainer, -0.2).withTimeout(backBallTime),
            WaitCommand(0.2).andThen(
                ConveyorCommand(robotContainer, 0.3))
        )
    ).withTimeout(timeout)