from commands2 import SubsystemBase
from photonvision import PhotonCamera, LEDMode
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard, SerialPort

from lib.sensors.nl_tof import NpTOF

class Vision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = PhotonCamera("gloworm")

        self.camera.setLEDMode(LEDMode.kOn)
        self.tof = NpTOF(SerialPort.Port.kUSB2)

    def log(self):
        SmartDashboard.putNumber("Vision Distance",  self.getDistance())
        SmartDashboard.putBoolean("Vision Has Target", self.camera.hasTargets())
        SmartDashboard.putNumber("Vision Yaw", self.getRotation2d().degrees())
        SmartDashboard.putData("Vision", self)

    def periodic(self):
        # self.log()
        pass

    def getDistance(self):
        res = self.tof.getDistance()

        # Usually it is impossible to get a distance of exact 0.0.
        
        if res == 0.0:
            return -1.0
        
        return res

    def getRotation2d(self):
        res = self.camera.getLatestResult()
        if res.hasTargets():
            return Rotation2d.fromDegrees(res.getBestTarget().getYaw())
        else:
            return Rotation2d(0)
