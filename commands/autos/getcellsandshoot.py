from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from commands import intakecommand, conveyorcommand, shootercommand


def IntakeConveyCommandGroup(robotContainer):
    return ParallelCommandGroup(
        intakecommand.IntakeCommand(robotContainer, 0.35),
        conveyorcommand.ConveyorCommand(robotContainer, 0.25),
    )


def AutoShootCommandGroup(robotContainer, shouldAutoRanging=False):
    return ParallelCommandGroup(
            shootercommand.ShooterCommand(
                robotContainer, shouldAutoRanging=shouldAutoRanging),

            SequentialCommandGroup(
                conveyorcommand.ConveyorCommand(robotContainer, -0.2).withTimeout(0.4),
                WaitCommand(0.8).andThen(conveyorcommand.ConveyorCommand(robotContainer, 0.3))
            )
        ).withTimeout(2.5)
