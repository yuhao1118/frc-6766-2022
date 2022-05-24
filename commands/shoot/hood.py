from commands2 import CommandBase
from lib.utils.interpolatedict import InterpolateDict

# Hood distance-angle map
# 射球罩距离-角度查找表
HoodDistanceAngle = InterpolateDict({
    0.0: 0.0,
    4.0: 20.0
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
        distance = self.robotContainer.visionControl.getDistanceMeters()
        angle = HoodDistanceAngle.getInterpolated(distance) if self.angle is None else float(self.angle)
        
        self.robotContainer.hoodDrive.setAngle(angle)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.hoodDrive.reset()
