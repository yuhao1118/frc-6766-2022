from commands.drivetrain.autoaimsimple import AutoAimSimple
from commands.drivetrain.autoaim import AutoAim
from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from commands.conveyor.conveyor import ConveyorCommand

from commands2 import ParallelCommandGroup, WaitUntilCommand, WaitCommand, InstantCommand


def AimCommand(robotContainer, aimMode, controller):
    if aimMode == "camera":
        return AutoAimSimple(robotContainer, controller=controller)
    elif aimMode == "odometry":
        return AutoAim(robotContainer, controller=controller)
    else:
        return InstantCommand()


def PrepareShootCommandGroup(robotContainer, aimMode, output=None, angle=None, controller=None):
    return ParallelCommandGroup(
        AimCommand(robotContainer, aimMode=aimMode, controller=controller),
        FlywheelCommand(robotContainer, output=output),
        HoodCommand(robotContainer, angle=angle),
    )


def PresetShoot(robotContainer, output=None, angle=None, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, output=output, angle=angle, aimMode="none")
    cmg.addCommands(
        WaitCommand(0.6).andThen(ConveyorCommand(robotContainer, 0.3))
    )

    return cmg.withTimeout(timeout)


def AutoShoot(robotContainer, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, aimMode="camera")
    cmg.addCommands(
        WaitUntilCommand(
            lambda: robotContainer.hoodDrive.isHoodReady() and robotContainer.flywheelDrive.isFlywheelReady()).andThen(
                ConveyorCommand(robotContainer, 0.3)
        )
    )

    return cmg.withTimeout(timeout)


def ManualShoot(robotContainer, timeout=2.0):
    return ConveyorCommand(robotContainer, 0.3).withTimeout(timeout)
