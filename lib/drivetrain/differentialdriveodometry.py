from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d
from wpilib import Timer
from collections import OrderedDict
import math


class VisionOdometry(DifferentialDriveOdometry):
    historyLengthSecs = 1.0
    visionShiftPerSec = 0.999
    visionMaxAngularVelocity = math.radians(180.0)
    nomainalFramerate = 22

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.driveDataCapacity = 100
        self.driveData = OrderedDict()

        # Flags
        self.baseLeftDistanceMeters = 0.0
        self.baseRightDistanceMeters = 0.0

    def setNominalFramerate(self, framerate):
        self.nomainalFramerate = framerate

    def addVisionMeasurement(self, timestamp, translation):
        historicalDrivePose, _, historicalAngularVelocity, _, _ = self.driveData.get(timestamp)
        if historicalDrivePose is None:
            print("No historical drive pose", timestamp)
            return

        angularErrorScale = abs(historicalAngularVelocity) / self.visionMaxAngularVelocity
        visionShift = 1 - math.pow(1 - self.visionShiftPerSec, 1 / self.nomainalFramerate)
        visionShift *= 1 - angularErrorScale

        currentPose, currentGyroRotation, _, currentLeftDistanceMeters, currentRightDistanceMeters = next(
            reversed(self.driveData.items()))

        # Estimate correct current pose
        fieldToVisionField = translation - historicalDrivePose.translation()
        visionLatencyCompFieldToTarget = Pose2d(
            currentPose.X() + fieldToVisionField.X(),
            currentPose.Y() + fieldToVisionField.Y(),
            currentPose.rotation()
        )
        filteredPose = Pose2d(
            currentPose.X() * (1 - visionShift) + visionLatencyCompFieldToTarget.X() * visionShift,
            currentPose.Y() * (1 - visionShift) + visionLatencyCompFieldToTarget.Y() * visionShift,
            currentPose.rotation() * (1 - visionShift)
        )

        # Update the odometry
        self.resetPosition(filteredPose, -currentGyroRotation)
        self.baseLeftDistanceMeters = currentLeftDistanceMeters
        self.baseRightDistanceMeters = currentRightDistanceMeters

    def updatePose(self, gyroRotation, gyroRateRadPerSec, leftDistanceMeters, rightDistanceMeters):
        timestamp = Timer.getFPGATimestamp()
        pose = super().update(gyroRotation,
                              leftDistanceMeters - self.baseLeftDistanceMeters,
                              rightDistanceMeters - self.baseRightDistanceMeters)
        self.driveData[timestamp] = (pose, gyroRotation, gyroRateRadPerSec, leftDistanceMeters, rightDistanceMeters)
        if len(self.driveData) > self.driveDataCapacity:
            self.driveData.popitem(last=False)

    def getTimestampedPose(self, timestamp):
        historicalDrivePose, _, _, _, _ = self.driveData.get(timestamp)
        return historicalDrivePose

    def getDistanceToTarget(self, target):
        return (self.getPose().translation() - target).norm()

    def getDriveRotation(self, timestamp):
        return self.driveData.get(timestamp)