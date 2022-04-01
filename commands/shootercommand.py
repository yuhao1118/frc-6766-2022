from commands2 import CommandBase

import constants

class ShooterCommand(CommandBase):
    """
    射球指令

    输入:
        robotContainer: RobotContainer实例
        output: 射球速度, 默认为在0cm处射球时所需电机速度
        shouldAutoRanging: 是否自动调整距离, 默认为False. 
        
        注: 此处的自动调整距离是指通过调节电机转速, 使得在当前速度下完成射球. 
        并非<自瞄并自动测距指令>中，通过移动底盘至目标距离开始射球.
    """
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
            range = self.robotContainer.visionControl.getDistance()
            range = int(range * 100)        # Convert to cm

            for key in constants.shooterSpeedHigh.keys():
                if range <= int(key.replace("cm", "")):
                    self.output = constants.shooterSpeedHigh[key]

        self.robotContainer.shooterDrive.setVelocity(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.shooterDrive.setVolts(0)
