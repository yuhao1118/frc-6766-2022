from commands2 import CommandBase


class CompressorCommand(CommandBase):
    """
    压缩机控制指令
    TODO: 切换至2020 API规范

    输入:
        robotContainer: RobotContainer实例
        enable: 是否启用压缩机, True为启用, False为关闭
    """
    def __init__(self, robotContainer, enable):
        super().__init__()
        super().setName("CompressorCommand")
        self.robotContainer = robotContainer
        self.enable = enable
        self.addRequirements(self.robotContainer.pneumaticControl)

    def execute(self):
        self.robotContainer.pneumaticControl.setCompressor(self.enable)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.pneumaticControl.setCompressor(False)
