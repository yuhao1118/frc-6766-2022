from commands2 import CommandBase

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.axisProfile import AxisProfile
from lib.utils.maths import clamp
from lib.enums.pov import POVEnum


class DriveCommand(CommandBase):
    """
    底盘遥控指令

    输入:
        robotContainer: RobotContainer实例
        io: 手柄控制io
    """

    def __init__(self, robotContainer, io):
        super().__init__()
        super().setName("DriveCommand")
        self.robotContainer = robotContainer
        self.linearXSupplier = io.getDriveXSupplier()
        self.angularZSupplier = io.getDriveZSupplier()
        self.povSupplier = io.getPOVSupplier()
        self.linearXProfiler = AxisProfile(0.04)
        self.angularZProfiler = AxisProfile(0.08)

        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):

        linearX = self.linearXProfiler.calculate(self.linearXSupplier())
        angularZ = self.angularZProfiler.calculate(self.angularZSupplier())
        povValue = self.povSupplier()

        arcadeSpeeds = WheelSpeedsPercentage.fromArcade(linearX, angularZ * constants.kDrivetrainTurnSensitive)
        curvatureSpeeds = WheelSpeedsPercentage.fromCurvature(linearX, angularZ)

        hybridScale = clamp(abs(linearX) / constants.kCurvatureThreshold, 0, 1)
        speeds = WheelSpeedsPercentage(curvatureSpeeds.left * hybridScale + arcadeSpeeds.left * (1 - hybridScale),
                                       curvatureSpeeds.right * hybridScale + arcadeSpeeds.right * (1 - hybridScale))

        if povValue == POVEnum.kUp:
            speeds = WheelSpeedsPercentage.fromArcade(0.2, 0.0)
        elif povValue == POVEnum.kDown:
            speeds = WheelSpeedsPercentage.fromArcade(-0.2, 0.0)
        elif povValue == POVEnum.kRight:
            speeds = WheelSpeedsPercentage.fromArcade(0.0, 0.2)
        elif povValue == POVEnum.kLeft:
            speeds = WheelSpeedsPercentage.fromArcade(0.0, -0.2)

        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.robotDrive.tankDrive(0.0, 0.0)
