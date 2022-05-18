from commands2 import CommandBase

import constants

# Shooter distance-speed map for high power cells
# 射球距离-速度查找表 (高球)
shooterSpeedHigh = {
    '0cm': 19.3,
    '35cm': 20,
    '75cm': 21.2,
    '100cm': 23,
    '150cm': 25.5
}

# Shooter distance-speed map for low power cells
# 射球距离-速度查找表 (低球)
shooterSpeedLow = {
    '0cm': 11.3
}


class FlywheelCommand(CommandBase):
    """
    射球指令

    输入:
        robotContainer: RobotContainer实例
        distance: 射出球的距离
        target: 射出球的目标: H/L (高/低)
        output=None: 射球速度,如提供则使用提供的速度,否则使用距离-速度查找表
    """
    def __init__(self,
                 robotContainer,
                 distance = 0.0,
                 target = "H",
                 output=None
                 ):

        super().__init__()
        super().setName("FlywheelCommand")
        self.robotContainer = robotContainer
        self.distance = distance
        self.target = target

        if output is not None:
            self.output = output
        else:
            if target == "H":
                self.output = shooterSpeedHigh[str(distance)]
            elif target == "L":
                self.output = shooterSpeedLow[str(distance)]

        self.addRequirements(self.robotContainer.shooterDrive)


    def execute(self):
        self.robotContainer.shooterDrive.setVelocity(self.output)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.shooterDrive.setVolts(0)
