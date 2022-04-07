from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard

import math

from lib.limelight.LEDMode import LEDMode
from lib.limelight.LimelightCamera import LimelightCamera

class Vision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = LimelightCamera()
        self.camera.setLEDMode(LEDMode.kOn)

    def log(self):
        SmartDashboard.putBoolean("Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Distance", self.getDistance())
        SmartDashboard.putNumber("Vision Yaw", self.getRotation2d().degrees())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        # self.log()
        pass

    def getDistance(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            angleToGoalDegrees = 41 + res.getBestTarget().getPitch()
            angleToGoalRadians = math.radians(angleToGoalDegrees)
            distance = (2.64 - 0.65) / math.tan(angleToGoalRadians)
            return distance

        return -1
    def getRotation2d(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            return Rotation2d.fromDegrees(res.getBestTarget().getYaw())
        else:
            return Rotation2d(0)
