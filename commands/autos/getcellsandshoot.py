from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from commands import intakecommand, conveyorcommand, shootercommand


def IntakeConveyCommandGroup(robotContainer):
    return ParallelCommandGroup(
        intakecommand.IntakeCommand(robotContainer, 0.35),
        conveyorcommand.ConveyorCommand(robotContainer, 0.25),
    )


def AutoShootCommandGroup(robotContainer, shouldAutoRanging=False, timeout=2.0, backBallTime=0.4):
    return ParallelCommandGroup(
        shootercommand.ShooterCommand(
            robotContainer, shouldAutoRanging=shouldAutoRanging),

        SequentialCommandGroup(
            conveyorcommand.ConveyorCommand(
                robotContainer, -0.2).withTimeout(backBallTime),
            WaitCommand(0.8).andThen(
                conveyorcommand.ConveyorCommand(robotContainer, 0.3))
        )
    ).withTimeout(timeout)
