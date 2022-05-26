from commands.drivetrain.driveandaim import DriveAimCommand
from commands.drivetrain.turntoangle import TurnToAngleCommand
from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from commands.conveyor.conveyor import ConveyorCommand

from commands2 import ParallelCommandGroup, WaitUntilCommand, WaitCommand, SequentialCommandGroup
from wpilib import RobotState


def RotateToTargetCommand(robotContainer):
    robotHeading = robotContainer.robotDrive.getPose(
        fromVisionOdometry=True).rotation()
    rotation = robotContainer.visionControl.getRobotToTargetTransform(
        robotHeading).rotation()

    return TurnToAngleCommand(robotContainer, rotation.radians())


def AimCommand(robotContainer):
    if not RobotState.isAutonomous():
        return SequentialCommandGroup(
            RotateToTargetCommand(robotContainer),
            DriveAimCommand(robotContainer)
        )
    else:
        return DriveAimCommand(robotContainer, controller=robotContainer.driverController)


def PrepareShootCommandGroup(robotContainer, output=None, angle=None):
    return ParallelCommandGroup(
        AimCommand(robotContainer),
        FlywheelCommand(robotContainer, output=output),
        HoodCommand(robotContainer, angle=angle),
    )


def PresetShootCommandGroup(robotContainer, output=None, angle=None, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, output=output, angle=angle)
    cmg.addCommands(
        WaitCommand(0.6).andThen(ConveyorCommand(robotContainer, 0.3))
    )

    return cmg.withTimeout(timeout)


def AutoShootCommandGroup(robotContainer, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer)
    cmg.addCommands(
        WaitUntilCommand(
            lambda: robotContainer.hoodDrive.isHoodReady() and robotContainer.flywheelDrive.isFlywheelReady()).andThen(
                ConveyorCommand(robotContainer, 0.3)
        )
    )

    return cmg.withTimeout(timeout)
