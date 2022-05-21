from commands2 import InstantCommand, SequentialCommandGroup, WaitCommand

def ResetHoodCommandGroup(robotContainer):
    """
    重置hood指令组
    """
    return SequentialCommandGroup(
        InstantCommand(robotContainer.hoodDrive.moveToBottom, [robotContainer.hoodDrive]),
        WaitCommand(0.1),
        InstantCommand(robotContainer.hoodDrive.reset, [robotContainer.hoodDrive]),
    )