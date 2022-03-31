from commands2 import ParallelCommandGroup, WaitCommand, SequentialCommandGroup
from commands import intakecommand, conveyorcommand, shootercommand

import constants

def IntakeConveyCommandGroup(robotContainer):
    return ParallelCommandGroup(
        intakecommand.IntakeCommand(robotContainer, 0.4),
        conveyorcommand.ConveyorCommand(robotContainer, 0.25),
    )


def AutoShootCommandGroup(robotContainer, shouldAutoRanging=False, timeout=2.2, backBallTime=0.3, output=constants.shooterSpeedHigh['0cm']):
    return ParallelCommandGroup(
        WaitCommand(0.1).andThen(shootercommand.ShooterCommand(
            robotContainer, output=output, shouldAutoRanging=shouldAutoRanging)),

        SequentialCommandGroup(
            conveyorcommand.ConveyorCommand(
                robotContainer, -0.2).withTimeout(backBallTime),
            WaitCommand(0.2).andThen(
                conveyorcommand.ConveyorCommand(robotContainer, 0.3))
        )
    ).withTimeout(timeout)
