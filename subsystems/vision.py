from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d
from wpimath.filter import LinearFilter
from wpilib import SmartDashboard

import math

from lib.limelight.LEDMode import LEDMode
from lib.limelight.LimelightCamera import LimelightCamera
from lib.limelight.LimelightUtils import estimateDistance
import constants


class Vision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = LimelightCamera()
        self.camera.setLEDMode(LEDMode.kOn)

        self.xOffsetFilter = LinearFilter.singlePoleIIR(
            constants.kVisionFilterTime, constants.kVisionFilterPeriod)
        self.distanceFilter = LinearFilter.singlePoleIIR(
            constants.kVisionFilterTime, constants.kVisionFilterPeriod)

        self.filteredDistanceMeters = 0.0
        self.filteredXOffsetRadians = 0.0

        self.lastValidDistance = 0.0

    def log(self):
        SmartDashboard.putBoolean(
            "Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Distance", self.getDistance())
        SmartDashboard.putNumber("Vision Yaw", self.getXOffset())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        # self.log()
        if self.camera.getLatestResult().hasTargets():
            target = self.camera.getLatestResult().getBestTarget()

            if target is not None:
                distance = estimateDistance(
                    math.radians(constants.kVisionCameraHeight),
                    math.radians(target.getPitch()),
                    constants.kVisionCameraHeight,
                    constants.kVisionTargetHeight,
                    target.getYaw()) + constants.kHubRadiusMeter

                self.filteredDistanceMeters = self.distanceFilter.calculate(
                    distance)
                self.filteredXOffsetRadians = self.xOffsetFilter.calculate(
                    target.getYaw())
                self.lastValidDistance = distance

            else:
                self.filteredDistanceMeters = self.distanceFilter.calculate(
                    self.lastValidDistance)
                self.filteredXOffsetRadians = 0.0
        else:
            self.filteredDistanceMeters = self.distanceFilter.calculate(
                self.lastValidDistance)
            self.filteredXOffsetRadians = 0.0

    def hasTargets(self):
        return self.camera.hasTargets()

    def getDistanceMeters(self):
        return self.filteredDistanceMeters

    def getXOffsetRadians(self):
        return self.filteredXOffsetRadians
