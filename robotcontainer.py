### Code Reference ###
# https://github.com/SteelRidgeRobotics/2021-2022_FRC_Season/tree/main/Captain%20Hook

from commands2 import RunCommand, ParallelCommandGroup, StartEndCommand
from commands2.button import JoystickButton
from wpilib import XboxController
from subsystems.drivetrain import Drivetrain
from subsystems.climber import Climber
from subsystems.shooter import Shooter
from subsystems.conveyor import Conveyor
from subsystems.intaker import Intaker

import constants
from trajectory.pathtrajectory import PathTrajectory


class RobotContainer:

    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled here or in the robot periodic methods, as this is a command-based system.
    The structure (commands, subsystems, and button mappings) should be done here.
    """

    def __init__(self):

        # Create the driver's controller.
        self.driverController = XboxController(constants.kDriverControllerPort)

        # Create instances of the subsystems.
        self.robotDrive = Drivetrain()
        self.climberDrive = Climber()
        self.shooterDrive = Shooter()
        self.conveyorDrive = Conveyor()
        self.intakerDrive = Intaker()

        # Configure and set the button bindings for the driver's controller.
        self.configureButtons()

        # Set the default command for the drive subsystem. It's default command will allow
        # the robot to drive with the controller.

        self.robotDrive.setDefaultCommand(
            RunCommand(
                lambda: self.robotDrive.arcadeDrive(
                    self.driverController.getRawAxis(
                        3) - self.driverController.getRawAxis(2),
                    self.driverController.getRawAxis(0) * 0.65,
                ),
                self.robotDrive,
            )
        )

        self.pathTrajectory = PathTrajectory()

    def getAutonomousCommand(self):
        trajectory = self.pathTrajectory.trajectoryForward
        trajectoryCommand = self.robotDrive.getTrajetoryCommand(trajectory)

        return trajectoryCommand

    def configureButtons(self):
        """Configure the buttons for the driver's controller"""

        intakeConveyCommandGroup = ParallelCommandGroup(
            StartEndCommand(
                lambda: self.intakerDrive.set(0.5),
                lambda: self.intakerDrive.set(0),
            ),
            StartEndCommand(
                lambda: self.conveyorDrive.set(0.25),
                lambda: self.conveyorDrive.set(0),
            )
        )

        (
            JoystickButton(self.driverController, XboxController.Button.kA)
            .whenPressed(lambda: self.shooterDrive.setVelocity(SmartDashboard.getNumber("ShooterV", 0)))
            .whenReleased(lambda: self.shooterDrive.setVolts(0))
        )

        (
            JoystickButton(self.driverController, XboxController.Button.kY)
            .whenPressed(intakeConveyCommandGroup)
        )

        (
            JoystickButton(self.driverController, XboxController.Button.kB)
            .whenPressed(lambda: self.conveyorDrive.set(-0.2))
            .whenReleased(lambda: self.conveyorDrive.set(0))
        )
