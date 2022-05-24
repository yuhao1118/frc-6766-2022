from commands2 import CommandBase


class ElevatorCommand(CommandBase):
    """
    爬升指令

    输入:
        robotContainer: RobotContainer实例
        output: 输出值, 取值范围-1.0~1.0, 正输入为爬升(<收>伸缩杆), 负输入为下降(<升>伸缩杆)
    """

    def __init__(self, robotContainer, output):
        super().__init__()
        super().setName("ElevatorCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.elevatorDrive)

    def execute(self):
        self.robotContainer.elevatorDrive.set(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        # self.robotContainer.elevatorDrive.holdAtHere()
        self.robotContainer.elevatorDrive.set(0.0)
