from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from commands import intakecommand, conveyorcommand, shootercommand

import constants

def IntakeConveyCommandGroup(robotContainer, reverse=False):
    """
    吸球传球并行指令组

    输入:
    1. robotContainer: RobotContainer实例
    2. [可选] reverse: 是否反向, 默认为False. 反向时表现为退球吐球

    输出: 
    并行指令组:
    1. Intake吸球, 速度0.4
    2. Conveyor传球, 速度0.25
    """
    return ParallelCommandGroup(
        intakecommand.IntakeCommand(robotContainer, -0.4 if reverse else 0.4),
        conveyorcommand.ConveyorCommand(robotContainer, -0.25 if reverse else 0.25),
    )


def AutoShootCommandGroup(robotContainer, shouldAutoRanging=False, timeout=2.2, backBallTime=0.3, output=shootercommand.shooterSpeedHigh['0cm']):
    """
    自动射球并行指令组:

    输入:
    1. robotContainer: RobotContainer实例
    2. shouldAutoRanging: 是否自动调整距离, 默认为False
    3. timeout: 总指令组超时时间, 默认为2.2秒
    4. backBallTime: Conveyor退球时间, 默认为0.3秒
    5. output: 射球速度, 默认为在0cm处射球时所需电机速度

    输出:
    并行指令组:
    1. 射球电机延迟<0.1s>启动, 因为需要时间给Conveyor退球
    2. 顺序指令组:
        2.1. Conveyor退球, 速度-0.2, 时间<backBallTime>
        2.2. Conveyor送球延迟<0.3s>启动, 速度0.3
    """
    return ParallelCommandGroup(
        WaitCommand(0.1).andThen(shootercommand.ShooterCommand(
            robotContainer, output=output, shouldAutoRanging=shouldAutoRanging)),

        SequentialCommandGroup(
            conveyorcommand.ConveyorCommand(
                robotContainer, -0.2).withTimeout(backBallTime),
            WaitCommand(0.2).andThen(
                conveyorcommand.ConveyorCommand(robotContainer, 0.3))
        )
    ).withTimeout(timeout)
