from commands2 import CommandBase


class IntakeCommand(CommandBase):
    """
    Intake吸球指令

    输入:
        robotContainer: RobotContainer实例
        output: 输出值, 取值范围-1.0~1.0, 正输入为吸球, 负输入为吐球
    """

    def __init__(self, robotContainer, output):
        super().__init__()
        super().setName("IntakeCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.intakerDrive)

    def execute(self):
        self.robotContainer.intakerDrive.set(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.intakerDrive.set(0)
