from commands2 import InstantCommand
from subsystems.elevator import ElevatorState


def ResetElevatorCommand(robotContainer):
    """
    重置elevator指令组
    """
    return InstantCommand(lambda: robotContainer.elevatorDrive.setState(ElevatorState.RESETTING))
