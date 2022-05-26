from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d
from wpimath.filter import LinearFilter
from wpilib import RobotBase, SmartDashboard

import math

from lib.limelight.LEDMode import LEDMode
from lib.limelight.LimelightCamera import LimelightCamera
from photonvision import PhotonUtils

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

        self.rawDistance = 0.0
        self.rawXOffsetRadians = 0.0

        self.lastValidDistance = 0.0
        self.latency = constants.kVisionLatencyMs
        self.isValid = False

    def log(self):
        SmartDashboard.putBoolean(
            "Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Distance (m)", self.getDistanceMeters())
        SmartDashboard.putNumber(
            "Vision Yaw (deg)", self.getXOffset().degrees())
        # SmartDashboard.putData("Vision", self)

    def periodic(self):
        self.log()
        res = self.camera.getLatestResult()
        if res.hasTargets():
            target = res.getBestTarget()
            self.isValid = True
            if target is not None:
                self.rawXOffsetRadians = math.radians(target.getYaw())

                self.rawDistance = PhotonUtils.calculateDistanceToTarget(
                    constants.kVisionCameraHeight,
                    constants.kVisionTargetHeight,
                    math.radians(constants.kVisionCameraPitch),
                    math.radians(target.getPitch()))

                if RobotBase.isSimulation():
                    self.rawDistance = SmartDashboard.getNumber("SimTargetDistance", 0.0)

                self.filteredDistanceMeters = self.distanceFilter.calculate(
                    self.rawDistance)
                self.filteredXOffsetRadians = self.xOffsetFilter.calculate(
                    self.rawXOffsetRadians)

                self.lastValidDistance = self.rawDistance
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

    def getRawDistanceMeters(self):
        return self.rawDistance

    def getRawXOffset(self):
        return Rotation2d(-self.rawXOffsetRadians)

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
                    self.getRawDistanceMeters(), self.getRawXOffset()),
                constants.kHubPose,
                robotHeading),
            constants.kHubPose,
            constants.kVisionCameraOffset
        )

    def getRobotToTargetTransform(self, robotHeading):
        return constants.kVisionCameraOffset + PhotonUtils.estimateCameraToTarget(
            PhotonUtils.estimateCameraToTargetTranslation(
                self.getRawDistanceMeters(), self.getRawXOffset()),
            constants.kHubPose,
            robotHeading)
