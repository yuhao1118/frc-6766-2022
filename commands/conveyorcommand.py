from commands2 import CommandBase


class ConveyorCommand(CommandBase):

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
