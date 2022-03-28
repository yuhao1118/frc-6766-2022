### Code Reference ###
# https://github.com/SteelRidgeRobotics/2021-2022_FRC_Season/tree/main/Captain%20Hook

from commands2.button import JoystickButton, POVButton
from wpilib import XboxController, SendableChooser, SmartDashboard
from subsystems.drivetrain import Drivetrain
from subsystems.climber import Climber
from subsystems.shooter import Shooter
from subsystems.conveyor import Conveyor
from subsystems.intaker import Intaker
from subsystems.pneumatic import Pneumatic
from subsystems.vision import Vision

import constants
from constants import POVEnum
from trajectory.pathtrajectory import PathTrajectory
from commands import pneumaticcommand, compressorcommand, intakecommand, conveyorcommand, drivecommand
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup
from commands.autos.getrangeandaim import GetRangeAndAimCommand


class RobotContainer:

    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled here or in the robot periodic methods, as this is a command-based system.
    The structure (commands, subsystems, and button mappings) should be done here.
    """

    def __init__(self):

        # Create the driver's controller.
        self.driverController = XboxController(constants.kDriverControllerPort)
        self.siderController = XboxController(constants.kSiderControllerPort)

        # Create instances of the subsystems.
        self.robotDrive = Drivetrain()
        self.climberDrive = Climber()
        self.shooterDrive = Shooter()
        self.conveyorDrive = Conveyor()
        self.intakerDrive = Intaker()
        self.pneumaticControl = Pneumatic()
        self.visionControl = Vision()

        # Configure and set the button bindings for the driver's controller.
        self.configureButtons()

        # Set the default command for the drive subsystem. It's default command will allow
        # the robot to drive with the controller.

        self.robotDrive.setDefaultCommand(
            drivecommand.DriveCommand(self, self.driverController)
        )

        self.pathTrajectory = PathTrajectory()

        self.autoChooser = SendableChooser()
        self.autoChooser.setDefaultOption("Forward", self.pathTrajectory.trajectoryForward)
        self.autoChooser.addOption("Backward", self.pathTrajectory.trajectoryForward)
        SmartDashboard.putData("Auto Chooser", self.autoChooser)


    def getAutonomousCommand(self):
        trajectory = self.autoChooser.getSelected()
        trajectoryCommand = self.robotDrive.getTrajetoryCommand(trajectory)

        return trajectoryCommand

    def configureButtons(self):
        """Configure the buttons for the driver's controller"""

        ############ Auto-assisted control ############

        # (Hold) Open/Close Intake
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kLeftBumper)
            .whileHeld(pneumaticcommand.PneumaticCommand(self, True))
        )

        # (Hold) Intake and convey
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kRightBumper)
            .whileHeld(IntakeConveyCommandGroup(self))
        )

        # (Press) Aiming
        (
            JoystickButton(self.siderController, XboxController.Button.kY)
            .whenPressed(GetRangeAndAimCommand(self).withTimeout(0.8))
        )

        # (Press) Shooting - fixed distance
        (
            JoystickButton(self.siderController, XboxController.Button.kB)
            .whenPressed(AutoShootCommandGroup(self))
        )

        # (Press) Shooting - distance from vision
        (
            JoystickButton(self.siderController, XboxController.Button.kX)
            .whenPressed(AutoShootCommandGroup(self, shouldAutoRanging=True))
        )

        ############ Manual Controls ############

        # (Hold) Intake Motor Backward
        (
            POVButton(self.siderController,
                      POVEnum.kUp)
            .whileHeld(intakecommand.IntakeCommand(self, -0.3))
        )

        # (Hold) Intake Motor Forward
        (
            POVButton(self.siderController,
                      POVEnum.kDown)
            .whileHeld(intakecommand.IntakeCommand(self, -0.3))
        )

        # (Toggle) Open/Close Compressor
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kA)
            .toggleWhenPressed(compressorcommand.CompressorCommand(self, True))
        )

        # (Hold) Conveyor Forward
        (
            POVButton(self.siderController,
                      POVEnum.kLeft)
            .whileHeld(conveyorcommand.ConveyorCommand(self, 0.3))
        )

        # (Hold) Conveyor Backward
        (
            POVButton(self.siderController,
                      POVEnum.kRight)
            .whileHeld(conveyorcommand.ConveyorCommand(self, -0.3))
        )
