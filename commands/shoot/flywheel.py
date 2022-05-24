from commands2 import CommandBase
from lib.utils.interpolatedict import InterpolateDict

# Shooter distance-speed map
# 射球距离-速度查找表
FlywheelDistanceAngle = InterpolateDict({
    0.0: 60.46,
    0.35: 62.66,
    0.75: 66.42,
    1.00: 72.06,
    1.50: 79.89
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
        distance = self.robotContainer.visionControl.getDistanceMeters()
        rps = FlywheelDistanceAngle.getInterpolated(distance) if self.output is None else float(self.output)

        self.robotContainer.flywheelDrive.setRPS(rps)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.flywheelDrive.reset()
