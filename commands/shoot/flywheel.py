from commands2 import CommandBase
from lib.utils.interpolatedict import InterpolateDict
from wpilib import SmartDashboard
from lib.utils.tunablenumber import TunableNumber

import constants

# Shooter distance-speed map
# 射球距离-速度查找表
FlywheelDistanceAngle = InterpolateDict({
    0.0: 19.3,
    0.35: 20,
    0.75: 21.2,
    1.00: 23,
    1.50: 25.5
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

        self.flywheelSpeed = TunableNumber("Flywheel/Speed", 70)
        self.addRequirements(self.robotContainer.shooterDrive)

    def initialize(self):
        distance = self.robotContainer.visionControl.getDistance()
        if self.output is None:
            self.output = FlywheelDistanceAngle.getInterpolated(distance) if distance is not None else 19.3

    def execute(self):
        self.robotContainer.shooterDrive.setRPS(float(self.flywheelSpeed))

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.shooterDrive.setVolts(0.0)
