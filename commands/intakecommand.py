from commands2 import CommandBase


class IntakeCommand(CommandBase):

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
