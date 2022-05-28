from commands2 import CommandBase

import constants
from lib.utils.interpolatedict import InterpolateDict

# 射球罩距离-角度查找表
HoodDistanceAngle = InterpolateDict({
    0.7: 0.0,       # 0.5
    0.85: 2.0,      # 0.7
    1.08: 3.0,      # 0.9
    1.25: 3.0,      # 1.1
    1.42: 4.0,      # 1.3
    1.58: 5.0,      # 1.5
    1.77: 7.0,      # 1.7
    1.91: 7.0,      # 1.9
    2.14: 8.0,      # 2.1
    2.33: 8.0,      # 2.3
    2.51: 9.0,      # 2.5
    2.68: 10.0,     # 2.7
    2.79: 11.0,     # 2.9
    2.95: 11.0,     # 3.1
})


class HoodCommand(CommandBase):
    """
    射球罩指令

    输入:
        robotContainer: RobotContainer实例
        angle=None: 射球罩角度, 如提供则使用提供的角度,否则使用距离-角度查找表
    """

    def __init__(self, robotContainer, angle=None):
        super().__init__()
        super().setName("HoodCommand")
        self.robotContainer = robotContainer
        self.angle = angle
        self.addRequirements(self.robotContainer.hoodDrive)

    def execute(self):
        distance = self.robotContainer.robotState.getDistanceToTarget(constants.kHubCenter)
        angle = HoodDistanceAngle.getInterpolated(distance) if self.angle is None else float(self.angle)

        self.robotContainer.hoodDrive.setAngle(angle)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.hoodDrive.reset()
