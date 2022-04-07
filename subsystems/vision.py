from commands2 import SubsystemBase
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard, SerialPort

from lib.limelight import LimelightCamera, LEDMode

class Vision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = LimelightCamera()
        self.camera.setLEDMode(LEDMode.kOn)

    def log(self):
        SmartDashboard.putBoolean("Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Yaw", self.getRotation2d().degrees())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        # self.log()
        pass

    def getDistance(self):
        return 0

    def getRotation2d(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            return Rotation2d.fromDegrees(res.getBestTarget().getYaw())
        else:
            return Rotation2d(0)
