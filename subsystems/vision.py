from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.filter import LinearFilter
from wpilib import SmartDashboard

import math

from lib.limelight.LEDMode import LEDMode
from lib.limelight.LimelightCamera import LimelightCamera
from photonvision import PhotonUtils

from trajectory.trajectory import Trajectory
import constants

VisionHub = Pose2d(8.23, 4.115, Rotation2d())


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
        self.latency = constants.kVisionLatencyMs
        self.isValid = False

    def log(self):
        SmartDashboard.putBoolean(
            "Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Distance (m)", self.getDistance())
        SmartDashboard.putNumber(
            "Vision Yaw (deg)", self.getXOffset().degrees())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        # self.log()
        res = self.camera.getLatestResult()
        if res.hasTargets():
            target = res.getBestTarget()
            self.isValid = True
            if target is not None:
                distance = PhotonUtils.calculateDistanceToTarget(
                    constants.kVisionCameraHeight,
                    constants.kVisionTargetHeight,
                    math.radians(constants.kVisionCameraPitch),
                    math.radians(target.getPitch())) * math.cos(self.getXOffset().radians()) + constants.kHubRadiusMeter

                self.filteredDistanceMeters = self.distanceFilter.calculate(
                    distance)
                self.filteredXOffsetRadians = self.xOffsetFilter.calculate(
                    target.getYaw())
                self.lastValidDistance = distance
                self.latency = res.getLatency() + constants.kVisionLatencyMs
            else:
                self.filteredDistanceMeters = self.distanceFilter.calculate(
                    self.lastValidDistance)
                self.filteredXOffsetRadians = 0.0
        else:
            self.filteredDistanceMeters = self.distanceFilter.calculate(
                self.lastValidDistance)
            self.filteredXOffsetRadians = 0.0
            self.isValid = False

    def hasTargets(self):
        return self.isValid

    def getDistanceMeters(self):
        return self.filteredDistanceMeters

    def getXOffset(self):
        return Rotation2d(-self.filteredXOffsetRadians)

    def getLatency(self):
        return self.latency

    def getRobotPose(self, robotHeading):
        return PhotonUtils.estimateFieldToRobot(
            PhotonUtils.estimateCameraToTarget(
                PhotonUtils.estimateCameraToTargetTranslation(
                    self.getDistanceMeters(), self.getXOffset()),
                VisionHub,
                robotHeading),
            VisionHub,
            constants.kVisionCameraOffset
        )

    def getRobotToTargetTransform(self, robotHeading):
        return constants.kVisionCameraOffset + PhotonUtils.estimateCameraToTarget(
            PhotonUtils.estimateCameraToTargetTranslation(
                self.getDistanceMeters(), self.getXOffset()),
            VisionHub,
            robotHeading)
