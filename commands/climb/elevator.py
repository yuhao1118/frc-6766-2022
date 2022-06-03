from commands2 import CommandBase

from subsystems.elevator import ElevatorState


class ElevatorCommand(CommandBase):
    """
    爬升指令

    输入:
        robotContainer: RobotContainer实例
        io: 手柄IO实例
    """

    def __init__(self, robotContainer, io):
        super().__init__()
        super().setName("ElevatorCommand")
        self.stateSwitchBtnSupplier = io.getClimbElevatorPressSupplier()
        self.elevatorDrive = robotContainer.elevatorDrive

    def execute(self):
        curState = self.elevatorDrive.getState()

        if self.stateSwitchBtnSupplier():
            if curState == ElevatorState.RESETTING:
                self.elevatorDrive.setState(ElevatorState.IDLE)
            elif curState == ElevatorState.IDLE:
                self.elevatorDrive.setState(ElevatorState.EXTENDING)
            elif curState == ElevatorState.EXTENDING:
                self.elevatorDrive.setState(ElevatorState.RETRACTING)
            elif curState == ElevatorState.RETRACTING or curState == ElevatorState.HOLDING:
                self.elevatorDrive.setState(ElevatorState.IDLE)

        if self.elevatorDrive.isRetracted() and self.elevatorDrive.getState() == ElevatorState.RETRACTING:
            self.elevatorDrive.setState(ElevatorState.HOLDING)

    def isFinished(self):
        return False

