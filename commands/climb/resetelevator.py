from commands2 import InstantCommand, SequentialCommandGroup, WaitCommand

def ResetElevatorCommand(robotContainer):
    """
    重置elevator指令组
    """
    return InstantCommand(robotContainer.elevatorDrive.reset, [robotContainer.elevatorDrive])