from commands2 import ParallelCommandGroup

from commands.intake.intake import IntakeCommand
from commands.conveyor.conveyor import ConveyorCommand

def AutoConveyCommandGroup(robotContainer, reverse=False):
    """
    吸球传球并行指令组

    输入:
    1. robotContainer: RobotContainer实例
    2. reverse=False: 是否反向, 默认为False. 反向时表现为退球吐球

    输出: 
    并行指令组:
    1. Intake吸球, 速度0.4
    2. Conveyor传球, 速度0.25
    """
    return ParallelCommandGroup(
        IntakeCommand(robotContainer, -0.4 if reverse else 0.4),
        ConveyorCommand(robotContainer, -0.25 if reverse else 0.25),
    )
