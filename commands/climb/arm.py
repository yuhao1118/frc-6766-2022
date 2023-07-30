from commands2 import CommandBase


class ArmCommand(CommandBase):
    """
    Arm气缸控制指令

    输入:
        robotContainer: RobotContainer实例
        enable: 是否打开Arm, True为打开, False为关闭
    """

    def __init__(self, robotContainer, enable):
        super().__init__()
        super().setName("ArmCommand")
        self.robotContainer = robotContainer
        self.enable = enable
        self.addRequirements(self.robotContainer.pneumaticControl)

    def initialize(self):
        self.robotContainer.pneumaticControl.setArm(self.enable)

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        #self.robotContainer.pneumaticControl.setArm(False)
        pass

