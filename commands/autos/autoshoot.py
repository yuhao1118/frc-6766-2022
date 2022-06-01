from commands.drivetrain.autoaimsimple import AutoAimSimple
from commands.drivetrain.autoaim import AutoAim
from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from commands.conveyor.conveyor import ConveyorCommand

from commands2 import ParallelCommandGroup, WaitUntilCommand, WaitCommand, PrintCommand


def AimCommand(robotContainer, aimMode, io, shouldAutoTerminate):
    if aimMode == "camera":
        return AutoAimSimple(robotContainer, io=io, shouldAutoTerminate=shouldAutoTerminate)
    elif aimMode == "odometry":
        return AutoAim(robotContainer, io=io, shouldAutoTerminate=shouldAutoTerminate)
    else:
        return PrintCommand("Unknown aim mode: {}".format(aimMode))


def PrepareShootCommandGroup(robotContainer, aimMode, shouldAutoTerminate, io=None, output=None, angle=None):
    return ParallelCommandGroup(
        AimCommand(robotContainer, aimMode, io=io, shouldAutoTerminate=shouldAutoTerminate),
        FlywheelCommand(robotContainer, output=output),
        HoodCommand(robotContainer, angle=angle),
    )


def PresetShoot(robotContainer, output=None, angle=None, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, aimMode="none", shouldAutoTerminate=True, output=output, angle=angle)
    cmg.addCommands(
        WaitCommand(0.6).andThen(ConveyorCommand(robotContainer, 0.3))
    )

    return cmg.withTimeout(timeout)


def AutoShoot(robotContainer, timeout=2.0):
    cmg = PrepareShootCommandGroup(robotContainer, aimMode="camera", shouldAutoTerminate=True)
    cmg.addCommands(
        WaitUntilCommand(
            lambda: robotContainer.hoodDrive.isHoodReady() and robotContainer.flywheelDrive.isFlywheelReady()).andThen(
                ConveyorCommand(robotContainer, 0.3)
        )
    )

    return cmg.withTimeout(timeout)


def ManualShoot(robotContainer, timeout=2.0):
    return ConveyorCommand(robotContainer, 0.3).withTimeout(timeout)
