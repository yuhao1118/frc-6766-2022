from commands2 import SubsystemBase
from photonvision import PhotonCamera, PhotonUtils, LEDMode
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard

import constants
import math

class Vision(SubsystemBase, PhotonCamera):
    def __init__(self):
        PhotonCamera.__init__(self, "photonvision")
        SubsystemBase.__init__(self)

        self.setLEDMode(LEDMode.kOn)

    def log(self):
        # inRange = (0 < self.getDistance() < 0.35)
        inRange = False
        SmartDashboard.putBoolean("ShootInRange", inRange)

    def periodic(self):
        self.log()

    def getDistance(self):
        res = self.getLatestResult()
        if res.hasTargets():
            return PhotonUtils.calculateDistanceToTarget(
                constants.kVisionCameraHeight,
                constants.kVisionTargetHeight,
                constants.kVisionCameraPitch,
                math.radians(res.getBestTarget().getPitch()))
        else:
            return -1

    def getRotation2d(self):
        res = self.getLatestResult()
        if res.hasTargets():
            return Rotation2d(res.getBestTarget().getYaw())
        else:
            return Rotation2d(0)