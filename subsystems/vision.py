from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d, Transform2d, Pose2d, Translation2d

import math

from lib.limelight.LEDMode import LEDMode
from lib.limelight.LimelightCamera import LimelightCamera
from lib.utils.circlefitter import CircleFitter

import constants


def sortCorners(corners, average):
    # Find top corners
    topLeftIndex, topRightIndex = None, None
    minPosPads, minNegRads = math.pi, math.pi
    for i in range(len(corners)):
        corner = corners[i]
        angleRad = (Rotation2d(corner[0] - average[0], average[1] - corner[1])
                    - Rotation2d.fromDegrees(90.0)).radians()
        if angleRad > 0.0:
            if angleRad < minPosPads:
                minPosPads = angleRad
                topLeftIndex = i
        else:
            if abs(angleRad) < minNegRads:
                minNegRads = abs(angleRad)
                topRightIndex = i

    # Find lower corners
    lowerIndex1, lowerIndex2 = None, None
    for i in range(len(corners)):
        alreadySaved = False
        if topLeftIndex == i or topRightIndex == i:
            alreadySaved = True
        if not alreadySaved:
            if lowerIndex1 is None:
                lowerIndex1 = i
            else:
                lowerIndex2 = i

    # Combine final list
    newCorners = []
    if topLeftIndex is not None:
        newCorners.append(corners[topLeftIndex])
    if topRightIndex is not None:
        newCorners.append(corners[topRightIndex])
    if lowerIndex1 is not None:
        newCorners.append(corners[lowerIndex1])
    if lowerIndex2 is not None:
        newCorners.append(corners[lowerIndex2])
    return newCorners


class Vision(SubsystemBase):
    circleFitPrecision = 0.01
    minTargetCount = 2  # For calculating odometry
    extraLatencySecs = 0.06  # Approximate camera + network latency
    vpw = 2.0 * math.tan(constants.kCameraFovHorizontal.radians() / 2.0)
    vph = 2.0 * math.tan(constants.kCameraFovVertical.radians() / 2.0)
    lastCaptureTimestamp = 0.0
    lastTranslations = []
    targetRes = None

    def __init__(self):
        super().__init__()
        self.camera = LimelightCamera()
        self.camera.setLEDMode(LEDMode.kOn)
        self.odometry = None

    def periodic(self):
        pipelineIndex = self.camera.getPipelineIndex()
        self.targetRes = self.camera.getLatestResult()
        if pipelineIndex == 1:
            targetCount = len(self.targetRes.getBestTarget().getCorners()) / 4
        else:
            targetCount = 0
        self.processFrame(targetCount)

    def setVisionOdometry(self, odometry):
        self.odometry = odometry

    def getXOffset(self):
        if self.targetRes.hasTargets():
            return Rotation2d.fromDegrees(self.targetRes.getBestTarget().getYaw())
        else:
            return Rotation2d()

    def hasTargets(self):
        return self.targetRes.hasTargets()

    def processFrame(self, targetCount):
        if self.targetRes.getCaptureTimestamp() == self.lastCaptureTimestamp:
            return

        if self.odometry is None:
            return

        captureTimestamp = round(self.targetRes.getCaptureTimestamp() - self.extraLatencySecs, 2)
        self.lastCaptureTimestamp = captureTimestamp

        if targetCount >= self.minTargetCount:
            cameraToTargetTranslations = []
            for targetIndex in range(targetCount):
                corners = self.targetRes.getBestTarget().getCorners()[targetIndex * 4, targetIndex * 4 + 4]
                totalX = sum(corners[i][0] for i in range(len(corners)))
                totalY = sum(corners[i][1] for i in range(len(corners)))

                targetAvg = (totalX / 4, totalY / 4)
                corners = sortCorners(corners, targetAvg)

                for i in range(len(corners)):
                    translation = self.solveCameraToTargetTranslation(
                        corners[i],
                        constants.kHubHeightHigher if i < 2 else constants.kHubHeightLower,
                    )
                    if translation is not None:
                        cameraToTargetTranslations.append(translation)

            self.lastTranslations = cameraToTargetTranslations

            # Combine corner translations to full target translation
            if len(cameraToTargetTranslations) >= self.minTargetCount * 4:
                cameraToTargetTranslation = CircleFitter.fit(constants.kHubRadiusMeter,
                                                             cameraToTargetTranslations,
                                                             self.circleFitPrecision)

                # Calculate field to robot translation
                cameraRotation = self.odometry.getDriveRotation(captureTimestamp)  # camera faces same as drivetrain
                if cameraRotation is not None:
                    fieldToTargetRotated = Transform2d(constants.kHubCenter, cameraRotation)
                    fieldToCamera = fieldToTargetRotated + Transform2d(-cameraToTargetTranslation, Rotation2d())
                    fieldToVehicle = fieldToCamera + constants.kCameraOffset.inverse()
                    fieldToVehicle = Pose2d(fieldToVehicle.translation(), fieldToVehicle.rotation())

                    if fieldToVehicle.X() > constants.kFieldLengthMeters \
                            or fieldToVehicle.X() < 0.0 \
                            or fieldToVehicle.Y() > constants.kFieldWidthMeters \
                            or fieldToVehicle.Y() < 0.0:
                        return

                    # Send final translation
                    self.odometry.addVisionMeasurement(captureTimestamp, fieldToVehicle.translation())

    def solveCameraToTargetTranslation(self, corner, goalHeight):
        halfWidthPixel = constants.kVisionWidthPixel / 2.0
        halfHeightPixel = constants.kVisionHeightPixel / 2.0
        nY = -((corner[0] - halfWidthPixel - constants.kVisionCrosshairX) / halfWidthPixel)
        nZ = -((corner[1] - halfHeightPixel - constants.kVisionCrosshairY) / halfHeightPixel)

        xzPlaneTranslation = Translation2d(1.0, self.vph / 2.0 * nZ).rotateBy(constants.kCameraPitch)
        x = xzPlaneTranslation.X()
        y = self.vpw / 2.0 * nY
        z = xzPlaneTranslation.Y()

        differentialHeight = constants.kCameraHeight - goalHeight
        if (z < 0.0) == (differentialHeight > 0.0):
            scaling = differentialHeight / -z
            distance = math.hypot(x, y) * scaling
            angle = Rotation2d(x, y)
            return Translation2d(distance * angle.cos(), distance * angle.sin())

        return None

    def setPipeline(self, pipeline):
        self.camera.setPipelineIndex(pipeline)
