from commands2 import CommandBase


class CompressorCommand(CommandBase):

    def __init__(self, robotContainer, enable):
        super().__init__()
        self.robotContainer = robotContainer
        self.enable = enable
        self.addRequirements(self.robotContainer.pneumaticControl)

    def execute(self):
        self.robotContainer.pneumaticControl.setCompressor(self.enable)

    def isFinished(self):
        return False

    def end(self):
        self.robotContainer.pneumaticControl.setCompressor(False)

    def interrupted(self):
        self.end()
