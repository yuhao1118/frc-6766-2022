from commands2 import SubsystemBase
from photonvision import PhotonCamera, PhotonUtils, LEDMode
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard

import constants
import math


class Vision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = PhotonCamera("gloworm")

        self.camera.setLEDMode(LEDMode.kOn)

    def log(self):
        SmartDashboard.putBoolean("ShootInRange",  (0 < self.getDistance() < 0.35))
        SmartDashboard.putBoolean("Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Yaw", self.getRotation2d().degrees())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        self.log()

    def getDistance(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            return PhotonUtils.calculateDistanceToTarget(
                constants.kVisionCameraHeight,
                constants.kVisionTargetHeight,
                constants.kVisionCameraPitch,
                math.radians(res.getBestTarget().getPitch()))
        else:
            return -1

    def getRotation2d(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            return Rotation2d.fromDegrees(res.getBestTarget().getYaw())
        else:
            return Rotation2d(0)
