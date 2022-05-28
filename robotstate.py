# Ref: https://github.com/Mechanical-Advantage/RobotCode2022

import math

from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d, Twist2d
from pytreemap import TreeMap

import constants


class RobotState:
    historyLengthSecs = 1.0
    visionShiftPerSec = 0.999
    visionMaxAngularVelocity = math.radians(180.0)

    driveData = TreeMap()
    visionData = TreeMap()

    basePose = Pose2d()
    lastestPose = Pose2d()

    lastDistanceMeters = 0.0
    lastRotation = Rotation2d()

    shouldResetOnNextVision = False

    def addDriveData(self, rotation, leftDistanceMeters, rightDistanceMeters):
        distanceMeters = (leftDistanceMeters + rightDistanceMeters) / 2.0
        self.driveData.put(Timer.getFPGATimestamp(),
                           Twist2d(
                               distanceMeters - self.lastDistanceMeters,
                               0.0,
                               (rotation - self.lastRotation).radians()
                           ))
        self.lastDistanceMeters = distanceMeters
        self.lastRotation = rotation
        self.update()

    def addVisionData(self, timestamp, translation):
        if self.shouldResetOnNextVision:
            self.resetPose(Pose2d(translation, self.getLatestRotation()))
            self.shouldResetOnNextVision = False
        else:
            self.visionData.put(timestamp, translation)
            self.update()

    def getLatestPose(self):
        return self.lastestPose

    def getLatestRotation(self):
        return self.lastestPose.rotation()

    def getDistanceToTarget(self, target):
        return (self.getLatestPose().translation() - target).norm()

    def resetPose(self, pose):
        self.basePose = pose
        self.driveData.clear()
        self.visionData.clear()
        self.update()

    def resetOnNextVision(self):
        self.shouldResetOnNextVision = True

    def getVisionResetComplete(self):
        return not self.shouldResetOnNextVision

    def update(self):
        # Clear old drive data
        while (self.driveData.size() > 1
               and self.driveData.first_key() < Timer.getFPGATimestamp() - self.historyLengthSecs):
            self.basePose = self.getPose(self.driveData.higher_key(self.driveData.first_key()))
            self.driveData.poll_first_entry()

        # Clear old vision data
        while (self.visionData.size() > 0
               and self.visionData.first_key() < self.driveData.first_key()):
            self.visionData.poll_first_entry()

        # Update latest pose
        self.lastestPose = self.getPose()

    def getPose(self, timestamp=None):
        # Get drive data in range
        filteredDriveData = self.driveData
        if timestamp is not None:
            filteredDriveData = self.driveData.head_map(timestamp)

        # Process drive and vision data
        pose = self.basePose
        for driveEntry in filteredDriveData.entry_set():
            nextTimestamp = self.driveData.higher_key(driveEntry.get_key())
            if nextTimestamp is None:
                nextTimestamp = driveEntry.get_key() + 0.02
            filteredVisionData = self.visionData.sub_map(driveEntry.get_key(), nextTimestamp)

            # Apply vision data
            for visionEntry in filteredVisionData.entry_set():
                angularVelocityRadPerSec = driveEntry.get_value().dtheta / (nextTimestamp - driveEntry.get_key())
                angularErrorScale = abs(angularVelocityRadPerSec) / self.visionMaxAngularVelocity
                visionShift = 1 - math.pow(1 - self.visionShiftPerSec, 1 / constants.kVisionNominalFramerate)
                visionShift *= 1 - angularErrorScale

                pose = Pose2d(
                    pose.X() * (1 - visionShift) + visionEntry.get_value().X() * visionShift,
                    pose.Y() * (1 - visionShift) + visionEntry.get_value().Y() * visionShift,
                    pose.rotation()
                )

            pose = pose.exp(driveEntry.get_value())

        return pose

    def getDriveRotation(self, timestamp):
        rotation = self.basePose.rotation()

        for entry in self.driveData.head_map(timestamp).entry_set():
            nextTimestamp = self.driveData.higher_key(entry.get_key())
            if nextTimestamp is not None and nextTimestamp > timestamp:
                t = (timestamp - entry.get_key()) / (nextTimestamp - entry.get_key())
                rotation = rotation + Rotation2d(entry.get_value().dtheta * t)
            else:  # Apply full twist
                rotation = rotation + Rotation2d(entry.get_value().dtheta)
        return rotation
