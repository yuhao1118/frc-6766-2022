from commands2 import CommandBase


class ClimbArmCommand(CommandBase):
    """
    爬升摇臂指令

    输入:
        robotContainer: RobotContainer实例
        output: 输出值, 取值范围-1.0~1.0, 正输入为摇臂朝车头方向摆动, 负输入为摇臂朝车尾方向摆动
    """

    def __init__(self, robotContainer, output):
        super().__init__()
        super().setName("ClimbArmCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.climberDrive)

    def execute(self):
        self.robotContainer.climberDrive.setArm(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.climberDrive.setArm(0)
