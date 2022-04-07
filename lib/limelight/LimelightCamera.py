# Ref: https://github.com/FRC703/Robotpy-Limelight/blob/master/limelight/limelight.py

from networktables import NetworkTables
from lib.limelight import LEDMode, LimelightPipelineResult, LimelightTrackedTarget
from wpimath.geometry import Transform2d, Translation2d, Rotation2d

class LimelightCamera:
    _enabled = 1
    _light = LEDMode.kOff
    _stream_mode = 0
    _snapshots = 0
    __nt = None
    _active_pipeline = 0

    def __init__(self, nt=None):
        """
        Represents a camera that is connected to limelight

        Args:
            nt: The NetworkTableInstance to pull data from. This can be a custom instance in 
            simulation, but should usually be the default NTInstance from NetworkTables.
        """
        if nt:
            self.__nt = nt
        else:
            self.__nt = NetworkTables.getTable("limelight")


    def getDriverMode(self):
        """
        Returns whether the camera is in driver mode.

        Returns:
            Whether the camera is in driver mode.
        """
        return self._enabled

    def getLEDMode(self):
        """
        Returns whether the camera is in driver mode.

        Returns:
            Whether the camera is in driver mode.
        """
        return self._light

    def getLatestResult(self):
        camtran = self.__nt.getNumberArray("camtran", [0, 0, 0, 0, 0, 0])
        pose = Transform2d(camtran[0], camtran[1], camtran[4])

        rawcorners =  self.__nt.getNumberArray("tcornxy", [0, 0, 0, 0, 0, 0, 0, 0])
        rawcorners = rawcorners[:8]
        
        corners = []

        for i in range(0, len(rawcorners), 2):
            corners.append((rawcorners[i], rawcorners[i+1]))

        target = LimelightTrackedTarget(
            self.__nt.getNumber("tx", 0),
            self.__nt.getNumber("ty", 0),
            self.__nt.getNumber("ta", 0),
            self.__nt.getNumber("ts", 0),
            pose, 
            corners)

        return LimelightPipelineResult(
            self.__nt.getNumber("tl", 0),
            target,
            self.hasTargets()
        )

    def getPipelineIndex(self):
        """
        Returns the active pipeline index.

        Returns:
            The active pipeline index.
        """
        return self._active_pipeline

    def hasTargets(self):
        """
        Whether the camera has found a valid target

        Returns:
            Any valid targets?
        """
        return bool(self.__nt.getNumber("tv", 0))

    def setDriverMode(self, driveMode):
        """
        Toggles driver mode.

        Args:
            driveMode: Whether to set driver mode.
        """
        self._enabled = driveMode

        if driveMode:
            self.__nt.putNumber("camMode", 1)
        else:
            self.__nt.putNumber("camMode", 0)

    def setLEDMode(self, status: LEDMode) -> None:
        """
        Set the status of the limelight lights

        Args:
            status: The status to set the light to
        """
        self._light = status
        self.__nt.putNumber(
            "ledMode", status.value if isinstance(status, LEDMode) else status
        )

    def setPipelineIndex(self, index):
        """
        Allows the user to select the active pipeline index.

        Args:
            index: The active pipeline index.
        """
        self._active_pipeline = index
        self.__nt.putNumber("pipeline", index)

