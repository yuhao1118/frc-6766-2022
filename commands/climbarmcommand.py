from commands2 import CommandBase


class ClimbArmCommand(CommandBase):

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
