from commands2 import CommandBase


class SoftLimitsCommand(CommandBase):
    """
    软限位指令: 是否打开软限位
    指令运行时为启用软限位, 指令结束时为关闭软限位


    输入:
        robotContainer: RobotContainer实例
    """

    def __init__(self, robotContainer):
        super().__init__()
        super().setName("SoftLimitsCommand")
        self.robotContainer = robotContainer
        self.addRequirements(self.robotContainer.climberDrive)

    def initialize(self):
        self.robotContainer.climberDrive.setSoftLimits(False)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.climberDrive.setSoftLimits(True)
