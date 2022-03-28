from commands2 import CommandBase

import constants

class ShooterCommand(CommandBase):
    def __init__(self,
                 robotContainer,
                 output=constants.shooterSpeedHigh['0cm'],
                 shouldAutoRanging=False
                 ):

        super().__init__()
        super().setName("ShooterCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.shouldAutoRanging=shouldAutoRanging
        self.addRequirements(self.robotContainer.shooterDrive, self.robotContainer.visionControl)


    def execute(self):
        if self.shouldAutoRanging:
            range = self.robotContainer.visionControl.getRange()
            range = int(range * 100)        # Convert to cm

            for key in constants.shooterSpeedHigh.keys():
                if range <= key:
                    self.output = constants.shooterSpeedHigh[key]

        self.robotContainer.shooterDrive.setVelocity(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.shooterDrive.setVolts(0)
