from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d
from wpilib import Timer
from collections import OrderedDict
import math

from lib.utils.maths import clamp


class DifferentialVOdometry(DifferentialDriveOdometry):
    historyLengthSecs = 1.0
    visionShiftPerSec = 0.85
    visionMaxAngularVelocity = math.radians(180.0)
    nomainalFramerate = 22

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.driveDataCapacity = 50
        self.driveData = OrderedDict()

        # Flags
        self.baseLeftDistanceMeters = 0.0
        self.baseRightDistanceMeters = 0.0

    def setNominalFramerate(self, framerate):
        self.nomainalFramerate = framerate

    def addVisionMeasurement(self, timestamp, translation):
        res = self.getDrivedata(timestamp)
        if res is None:
            return

        latestDriveData = next(reversed(self.driveData.items()))
        if latestDriveData is None:
            return

        historicalDrivePose, _, historicalAngularVelocity, _, _ = res
        currentPose, currentGyroRotation, _, currentLeftDistanceMeters, currentRightDistanceMeters = latestDriveData[1]

        angularErrorScale = clamp(abs(historicalAngularVelocity) / self.visionMaxAngularVelocity, 0.0, 1.0)
        visionShift = 1 - math.pow(1 - self.visionShiftPerSec, 1 / self.nomainalFramerate)
        visionShift *= 1 - angularErrorScale

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
        self.resetPosition(filteredPose, currentGyroRotation)
        self.baseLeftDistanceMeters = currentLeftDistanceMeters
        self.baseRightDistanceMeters = currentRightDistanceMeters

    def updatePose(self, gyroRotation, gyroRateRadPerSec, leftDistanceMeters, rightDistanceMeters):
        timestamp = round(Timer.getFPGATimestamp(), 2)
        pose = super().update(gyroRotation,
                              leftDistanceMeters - self.baseLeftDistanceMeters,
                              rightDistanceMeters - self.baseRightDistanceMeters)
        self.driveData[timestamp] = (pose, gyroRotation, gyroRateRadPerSec, leftDistanceMeters, rightDistanceMeters)
        if len(self.driveData) > self.driveDataCapacity:
            self.driveData.popitem(last=False)

    def getDrivedata(self, timestamp):
        res = self.driveData.get(timestamp)
        if res is None:
            for i in self.driveData.keys():
                if timestamp - i <= 0.01:
                    return self.driveData[i]
            print("No historical drive pose", timestamp)
        else:
            return res

    def getDistanceToTarget(self, target):
        return (self.getPose().translation() - target).norm()

    def getDriveRotation(self, timestamp):
        res = self.getDrivedata(timestamp)
        if res is not None:
            return res[0].rotation()
