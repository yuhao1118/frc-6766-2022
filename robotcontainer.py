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
from enums.pov import POVEnum
from trajectory.trajectory import Trajectory
from commands import pneumaticcommand, compressorcommand, intakecommand, conveyorcommand, drivecommand, shootercommand, climbarmcommand
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup
from commands.autos.getrangeandaim import GetRangeAndAimCommand
from commands.autos.autopath import Auto1CommandGroup, TestCommandGroup


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

        # Create instances of the commands in SmartDashboard.
        self.putCommandsToSmartDashboard()

        # Configure and set the button bindings for the driver's controller.
        self.configureButtons()

        # Set the default command for the drive subsystem. It's default command will allow
        # the robot to drive with the controller.

        self.robotDrive.setDefaultCommand(
            drivecommand.DriveCommand(self, self.driverController)
        )

        self.autoChooser = SendableChooser()
        self.autoChooser.setDefaultOption(
            "Auto1", Auto1CommandGroup(self))
        self.autoChooser.addOption(
            "Test Forward", TestCommandGroup(self, Trajectory.ForwardTest))
        self.autoChooser.addOption(
            "Test Backward", TestCommandGroup(self, Trajectory.BackwardTest))
        self.autoChooser.addOption(
            "Test Auto11", TestCommandGroup(self, Trajectory.Auto11))
        self.autoChooser.addOption(
            "Test Auto12", TestCommandGroup(self, Trajectory.Auto12))
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def putCommandsToSmartDashboard(self):
        """Put commands to the SmartDashboard (Test Only)"""
        SmartDashboard.putData("Open Intake", pneumaticcommand.PneumaticCommand(self, True))
        SmartDashboard.putData("Close Intake", pneumaticcommand.PneumaticCommand(self, False))
        SmartDashboard.putData("Intake and Convey", IntakeConveyCommandGroup(self))
        SmartDashboard.putData("Auto-assisted Shoot (Fixed)", AutoShootCommandGroup(self))
        SmartDashboard.putData("Auto-assisted Shoot (Dynamic)", AutoShootCommandGroup(self, shouldAutoRanging=True))
        SmartDashboard.putData("Auto-assisted Aim", GetRangeAndAimCommand(self))
        SmartDashboard.putData("Auto-assisted Ranging and Aim", GetRangeAndAimCommand(self, shouldAutoRanging=True))
        SmartDashboard.putData("Intake motor forward", intakecommand.IntakeCommand(self, 0.3))
        SmartDashboard.putData("Intake motor backward", intakecommand.IntakeCommand(self, -0.3))
        SmartDashboard.putData("Conveyor motor forward", conveyorcommand.ConveyorCommand(self, 0.3))
        SmartDashboard.putData("Conveyor motor backward", conveyorcommand.ConveyorCommand(self, -0.3))
        SmartDashboard.putData("Shooter Forward High", shootercommand.ShooterCommand(self, output=constants.shooterSpeedHigh['0cm']))
        SmartDashboard.putData("Shooter Forward Low", shootercommand.ShooterCommand(self, output=constants.shooterSpeedLow['0cm']))
        SmartDashboard.putData("Shooter Backward", shootercommand.ShooterCommand(self, output=-5.0))



    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()

    def configureButtons(self):
        """Configure the buttons for the driver's controller"""

        ############ Auto-assisted control ############

        # (Hold) (Sider) (LB) Open/Close Intake
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kLeftBumper)
            .whileHeld(pneumaticcommand.PneumaticCommand(self, True))
        )

        # (Hold) (Sider) (RB) Intake and convey
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kRightBumper)
            .whileHeld(IntakeConveyCommandGroup(self))
        )

        # (Press) (Driver) (Y) Aiming
        (
            JoystickButton(self.driverController, XboxController.Button.kY)
            .whenPressed(GetRangeAndAimCommand(self).withTimeout(0.8))
        )

        # (Press) (Driver) (B) Shooting - high - fixed distance
        (
            JoystickButton(self.driverController, XboxController.Button.kB)
            .whenPressed(AutoShootCommandGroup(self))
        )

        # (Press) (Driver) (X) Shooting - low
        (
            JoystickButton(self.siderController, XboxController.Button.kX)
            .whenPressed(AutoShootCommandGroup(self, output=constants.shooterSpeedLow['0cm']))
        )

        ############ Manual Controls ############

        # (Hold) (Sider) (POV-Up) Intake Motor Forward
        (
            POVButton(self.siderController,
                      POVEnum.kUp)
            .whileHeld(intakecommand.IntakeCommand(self, 0.4))
        )

        # (Hold) (Sider) (POV-Down) Intake Motor Backward
        (
            POVButton(self.siderController,
                      POVEnum.kDown)
            .whileHeld(intakecommand.IntakeCommand(self, -0.4))
        )

        # (Toggle) (Sider) (Back) Open/Close Compressor
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kBack)
            .toggleWhenPressed(compressorcommand.CompressorCommand(self, True))
        )

        # (Hold) (Sider) (POV-Right) Arm Forward
        (
            POVButton(self.siderController,
                      POVEnum.kRight)
            .whileHeld(climbarmcommand.ClimbArmCommand(self, 0.3))
        )

        # (Hold) (Sider) (POV-Left) Arm Backward
        (
            POVButton(self.siderController,
                        POVEnum.kLeft)
            .whileHeld(climbarmcommand.ClimbArmCommand(self, -0.3))
        )

        # (Hold) (Sider) (A) Conveyor Forward
        (
            JoystickButton(self.siderController,
                      XboxController.Button.kA)
            .whileHeld(conveyorcommand.ConveyorCommand(self, 0.3))
        )

        # (Hold) (Sider) (Y) Conveyor Backward
        (
            JoystickButton(self.siderController,
                      XboxController.Button.kY)
            .whileHeld(conveyorcommand.ConveyorCommand(self, -0.3))
        )

        # (Hold) (Sider) (B) Climb Up
        (
            JoystickButton(self.siderController,
                        XboxController.Button.kB)
            .whileHeld(climbcommand.ClimbCommand(self, 0.3))
        )

        # (Hold) (Sider) (X) Climb Down 
        (
            JoystickButton(self.siderController,
                        XboxController.Button.kX)
            .whileHeld(climbcommand.ClimbCommand(self, -0.3))
        )
