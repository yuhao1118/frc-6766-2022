from commands2 import InstantCommand


def ResetElevatorCommand(robotContainer):
    """
    重置elevator指令组
    """
    return InstantCommand(robotContainer.elevatorDrive.reset, [robotContainer.elevatorDrive])
