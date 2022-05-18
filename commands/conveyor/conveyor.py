from commands2 import CommandBase


class ConveyorCommand(CommandBase):
    """
    传送带指令

    输入:
        robotContainer: RobotContainer实例
        output: 输出值, 取值范围-1.0~1.0, 正输入为传送带送球至炮台, 负输入为传送带退球至Intake
    """

    def __init__(self, robotContainer, output):
        super().__init__()
        super().setName("ConveyorCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.conveyorDrive)

    def execute(self):
        self.robotContainer.conveyorDrive.set(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.conveyorDrive.set(0)
