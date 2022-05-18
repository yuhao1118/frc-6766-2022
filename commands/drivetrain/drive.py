# Implement custom curvature drive
# Ref: https://github.com/Mechanical-Advantage/RobotCode2022

from commands2 import CommandBase
from wpilib import SmartDashboard, XboxController
from wpimath.controller import PIDController

import constants
from lib.drivetrain.wheelspeedspercentage import WheelSpeedsPercentage
from lib.utils.math import clamp, axisProfile
from lib.enums.drivemode import DriveModeEnum
from lib.enums.pov import POVEnum

class DriveCommand(CommandBase):
    """
    底盘遥控指令

    输入:
        robotContainer: RobotContainer实例
        controller: 底盘Xbox遥控器实例
    """

    def __init__(self, robotContainer, controller, driveMode = DriveModeEnum.ArcadeDrive):
        super().__init__()
        super().setName("DriveCommand")
        self.robotContainer = robotContainer
        self.controller = controller
        self.driveMode = driveMode
        self.turnPidController = PIDController(
            constants.kPVisionTurn,
            constants.kIVisionTurn,
            constants.kDVisionTurn
        )
        self.turnPidController.setTolerance(positionTolerance=2.0)
        self.addRequirements(self.robotContainer.robotDrive)

    def execute(self):
        linearX =  axisProfile(-self.controller.getRawAxis(1))
        angularZ =  axisProfile(self.controller.getRawAxis(4))
        povValue = self.controller.getPOV()

        speeds = WheelSpeedsPercentage(0, 0)

        if self.driveMode == DriveModeEnum.ArcadeDrive:
            speeds = WheelSpeedsPercentage.fromArcade(linearX, angularZ * constants.kDrivetrainTurnSensitive)

        elif self.driveMode == DriveModeEnum.CurvatureDrive:
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

        # Hold RB to slow down
        if self.controller.getRawButton(XboxController.Button.kRightBumper):
            speeds *= 0.5

        # Hold LB to drive to the target
        if self.controller.getRawButton(XboxController.Button.kLeftBumper):
            angle = self.robotContainer.visionControl.getRotation2d().degrees()
            turnSpeed = -self.turnPidController.calculate(angle, 0.0)
            if self.turnPidController.atSetpoint():
                turnSpeed = 0.0

            speeds = speeds + WheelSpeedsPercentage.fromArcade(0.0, turnSpeed)

        self.robotContainer.robotDrive.tankDrive(speeds.left, speeds.right)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.robotContainer.robotDrive.tankDrive(0.0, 0.0)
