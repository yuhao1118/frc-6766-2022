# Implement custom curvature drive
# Ref: https://github.com/Mechanical-Advantage/RobotCode2022

from commands2 import CommandBase

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.maths import clamp, axisProfile
from lib.enums.pov import POVEnum

class DriveCommand(CommandBase):
    """
    底盘遥控指令

    输入:
        robotContainer: RobotContainer实例
        controller: 底盘Xbox遥控器实例
    """

    def __init__(self, robotContainer, controller):
        super().__init__()
        super().setName("DriveCommand")
        self.robotContainer = robotContainer
        self.controller = controller
        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        linearX =  axisProfile(-self.controller.getRawAxis(1))
        angularZ =  axisProfile(self.controller.getRawAxis(4))
        povValue = self.controller.getPOV()

        arcadeSpeeds = WheelSpeedsPercentage.fromArcade(linearX, angularZ * constants.kDrivetrainTurnSensitive)
        curvatureSpeeds = WheelSpeedsPercentage.fromCurvature(linearX, angularZ)

        hybridScale = clamp(abs(linearX) / constants.kCurvatureThreshold, 0, 1)
        speeds = WheelSpeedsPercentage(
            curvatureSpeeds.left * hybridScale
                + arcadeSpeeds.left * (1 - hybridScale),
            curvatureSpeeds.right * hybridScale
                + arcadeSpeeds.right * (1 - hybridScale))

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
