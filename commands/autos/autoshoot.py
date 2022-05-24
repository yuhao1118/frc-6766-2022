from commands.drivetrain.driveandaim import DriveAimCommand
from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from commands.conveyor.conveyor import ConveyorCommand

from commands2 import ParallelCommandGroup, WaitUntilCommand, WaitCommand


def PrepareShootCommandGroup(robotContainer, output=None, angle=None):
    return ParallelCommandGroup(
        DriveAimCommand(
            robotContainer, controller=robotContainer.driverController),
        FlywheelCommand(robotContainer, output=output),
        HoodCommand(robotContainer, angle=angle),
    )


def ManualShootCommandGroup(robotContainer, output=None, angle=None, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, output=output, angle=angle)
    cmg.addCommands(
        WaitCommand(0.3).andThen(ConveyorCommand(robotContainer, 0.3))
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
