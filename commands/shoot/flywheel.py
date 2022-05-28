from commands2 import CommandBase

import constants
from lib.utils.interpolatedict import InterpolateDict

# 射球距离-速度查找表
FlywheelDistanceAngle = InterpolateDict({
    0.7: 57.5,          # 0.5
    0.85: 60.0,         # 0.7
    1.08: 60.0,         # 0.9
    1.25: 61.0,         # 1.1
    1.42: 62.5,         # 1.3
    1.58: 63.0,         # 1.5
    1.77: 65.0,         # 1.7
    1.91: 67.0,         # 1.9
    2.14: 67.5,         # 2.1
    2.33: 68.5,         # 2.3
    2.51: 70.0,         # 2.5
    2.68: 71.0,         # 2.7
    2.79: 72.0,         # 2.9
    2.95: 73.5,         # 3.1
})


class FlywheelCommand(CommandBase):
    """
    射球指令

    输入:
        robotContainer: RobotContainer实例
        output=None: 射球速度,如提供则使用提供的速度,否则使用距离-速度查找表
    """

    def __init__(self,
                 robotContainer,
                 output=None
                 ):

        super().__init__()
        super().setName("FlywheelCommand")
        self.robotContainer = robotContainer
        self.output = output
        self.addRequirements(self.robotContainer.flywheelDrive)

    def execute(self):
        distance = self.robotContainer.robotState.getDistanceToTarget(constants.kHubCenter)
        rps = FlywheelDistanceAngle.getInterpolated(distance) if self.output is None else float(self.output)

        self.robotContainer.flywheelDrive.setRPS(rps)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.flywheelDrive.reset()
