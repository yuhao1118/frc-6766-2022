from commands2 import CommandBase


class ClimbCommand(CommandBase):

    def __init__(self, robotContainer, output):
        super().__init__()
        super().setName("ClimbCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.climberDrive)

    def execute(self):
        self.robotContainer.climberDrive.set(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.climberDrive.set(0)
