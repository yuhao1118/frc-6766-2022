### Code Reference ###
# https://github.com/SteelRidgeRobotics/2021-2022_FRC_Season/tree/main/Captain%20Hook

from commands2.button import JoystickButton, POVButton
from commands2 import ParallelCommandGroup
from wpilib import XboxController, SendableChooser, SmartDashboard
from subsystems.drivetrain import Drivetrain
from subsystems.elevator import Elevator
from subsystems.arm import Arm
from subsystems.flywheel import Flywheel
from subsystems.conveyor import Conveyor
from subsystems.intaker import Intaker
from subsystems.pneumatic import Pneumatic
from subsystems.vision import Vision
from subsystems.hood import Hood

import constants
from lib.enums.pov import POVEnum
from trajectory.trajectory import Trajectory

# from commands.drivetrain.driveandaim import DriveAimCommand
from commands.drivetrain.drive import DriveCommand

from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.resethood import ResetHoodCommandGroup

from commands.intake.intake import IntakeCommand
from commands.intake.compressor import CompressorCommand
from commands.intake.pneumatic import PneumaticCommand

from commands.conveyor.conveyor import ConveyorCommand

from commands.climb.elevator import ElevatorCommand
from commands.climb.arm import ArmCommand
from commands.climb.resetelevator import ResetElevatorCommand

from commands.autos.autopath import Auto1CommandGroup, Auto2CommandGroup, Auto3CommandGroup, TestCommandGroup
from commands.autos.autoshoot import PrepareShootCommandGroup
from commands.autos.autoconvey import AutoConveyCommandGroup


class RobotContainer:
    """
    This file manages all the subsystems and the lifecycle of each command. 
    Joystick - command bindings are also configurated here.

    在这个文件中管理所有子系统和每个指令的生命周期. 手柄按键与对应指令的绑定也在这个文件中设置.
    """

    def __init__(self):

        # Create the driver's controller.
        # 创建手柄实例.
        self.driverController = XboxController(constants.kDriverControllerPort)
        self.siderController = XboxController(constants.kSiderControllerPort)

        # Create instances of the subsystems.
        # 创建各子系统实例.
        self.robotDrive = Drivetrain()
        self.elevatorDrive = Elevator()
        self.armDrive = Arm()
        self.flywheelDrive = Flywheel()
        self.hoodDrive = Hood()
        self.conveyorDrive = Conveyor()
        self.intakerDrive = Intaker()
        self.pneumaticControl = Pneumatic()
        self.visionControl = Vision()

        # Create instances of the commands in SmartDashboard.
        # 在仪表盘显示各个指令(组), 用于调试.
        self.putCommandsToSmartDashboard()

        # Configure and set the button bindings for the driver's controller.
        # 设置手柄按键与对应指令的绑定.
        self.configureButtons()

        # Set the default command for the drive subsystem. It's default command will allow
        # the robot to drive with the controller.
        # 设置底盘默认指令, 允许机器人使用手柄控制.
        self.robotDrive.setDefaultCommand(
            DriveCommand(self, self.driverController)
        )

        # Display the autonomous chooser on the SmartDashboard.
        # 在仪表盘显示自动阶段任务下拉选择器.
        self.autoChooser = SendableChooser()

        self.autoChooser.setDefaultOption(
            "Auto1", Auto1CommandGroup(self))                               # 3 ball auto. 3球自动
        self.autoChooser.addOption(
            "Auto2", Auto2CommandGroup(self))                               # 2 ball auto. 2球自动
        self.autoChooser.addOption(
            "Auto3", Auto3CommandGroup(self))                               # 2 ball auto + 1 defense ball. 2球自动 + 干扰敌方一球

        # The rest are test trajectories.
        # 剩下的是测试轨迹.
        self.autoChooser.addOption(
            "Test Forward", TestCommandGroup(self, Trajectory.ForwardTest))
        self.autoChooser.addOption(
            "Test Backward", TestCommandGroup(self, Trajectory.BackwardTest))
        self.autoChooser.addOption(
            "Test Auto11", TestCommandGroup(self, Trajectory.Auto11))
        self.autoChooser.addOption(
            "Test Auto12", TestCommandGroup(self, Trajectory.Auto12))
        self.autoChooser.addOption(
            "Test Auto2",  TestCommandGroup(self, Trajectory.Auto2))
        self.autoChooser.addOption(
            "Test Auto31", TestCommandGroup(self, Trajectory.Auto31))
        self.autoChooser.addOption(
            "Test Auto32", TestCommandGroup(self, Trajectory.Auto32))

        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def putCommandsToSmartDashboard(self):
        """Put commands to the SmartDashboard (Test Only)"""
        SmartDashboard.putData("Open Intake", PneumaticCommand(self, True))
        SmartDashboard.putData("Close Intake", PneumaticCommand(self, False))
        SmartDashboard.putData("Intake and Convey",
                               AutoConveyCommandGroup(self))
        SmartDashboard.putData("Auto-assisted Aim",
                               PrepareShootCommandGroup(self))
        SmartDashboard.putData("Intake motor forward",
                               IntakeCommand(self, 0.3))
        SmartDashboard.putData("Intake motor backward",
                               IntakeCommand(self, -0.3))
        SmartDashboard.putData("Conveyor motor forward",
                               ConveyorCommand(self, 0.3))
        SmartDashboard.putData("Conveyor motor backward",
                               ConveyorCommand(self, -0.3))
        SmartDashboard.putData("Flywheel Forward High",
                               FlywheelCommand(self, output=19.3))
        SmartDashboard.putData("Flywheel Forward Low",
                               FlywheelCommand(self, output=11.3))
        SmartDashboard.putData("Flywheel Backward",
                               FlywheelCommand(self, output=-5.0))

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()

    def getResetCommand(self):
        return ParallelCommandGroup(ResetHoodCommandGroup(self), ResetElevatorCommand(self))

    def configureButtons(self):
        """Configure the buttons for the driver's controller"""

        ############ Auto-assisted control / 自动辅助控制 ############

        # (Hold) (Sider) (LB) Open/Close Intake
        # (按住) (副操作手) (LB) 开/关 Intake, 按住时打开Intake, 松手自动收起
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kLeftBumper)
            .whileHeld(PneumaticCommand(self, True))
        )

        # (Hold) (Sider) (RB) Intake and convey
        # (按住) (副操作手) (RB) 吸球并传球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kRightBumper)
            .whileHeld(AutoConveyCommandGroup(self))
            .whenReleased(ConveyorCommand(self, -0.2).withTimeout(0.20))
        )

        # (Hold) (Sider) (Start) Backball and drop
        # (按住) (副操作手) (Start) 退球并吐球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kStart)
            .whileHeld(AutoConveyCommandGroup(self, reverse=True))
        )

        # (Press) (Sider) (Y) Aiming
        # (切换) (副操作手) (Y) 自瞄
        (
            JoystickButton(self.siderController, XboxController.Button.kY)
            .toggleWhenPressed(PrepareShootCommandGroup(self))
        )

        # (Hold) (Drive) (LB) Aiming
        # (按住) (主操作手) (LB) 自瞄 (同时可以前后移动)
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kLeftBumper)
            .whileHeld(PrepareShootCommandGroup(self))
        )

        # (Press) (Sider) (B) Shooting - fixed distance at 0cm
        # (按一下) (副操作手) (B) 射球
        (
            JoystickButton(self.siderController, XboxController.Button.kB)
            .whenPressed(ConveyorCommand(self, 0.3).withTimeout(2.0))
        )

        ############ Manual Controls ############

        # (Hold) (Driver) (B) Climb Up
        # (按住) (主操作手) (B) 摇臂向车头
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kB)
            .whileHeld(ArmCommand(self, 0.15))
        )

        # (Hold) (Driver) (X) Climb Up
        # (按住) (主操作手) (X) 摇臂向车尾
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kX)
            .whileHeld(ArmCommand(self, -0.15))
        )

        # (Hold) (Driver) (Y) Climb Up
        # (按住) (主操作手) (Y) 爬升, <缩>伸缩杆
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kY)
            .whileHeld(ElevatorCommand(self, 1.0))
        )

        # (Hold) (Driver) (A) Climb Down
        # (按住) (主操作手) (A) 爬升, <升>伸缩杆
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kA)
            .whileHeld(ElevatorCommand(self, -1.0))
        )

        # (Hold) (Sider) (POV-Left) Intake Motor Forward
        # (按住) (副操作手) (POV左) Intake吸球
        (
            POVButton(self.siderController,
                      POVEnum.kLeft)
            .whileHeld(IntakeCommand(self, 0.4))
        )

        # (Hold) (Sider) (POV-Right) Intake Motor Backward
        # (按住) (副操作手) (POV右) Intake吐球
        (
            POVButton(self.siderController,
                      POVEnum.kRight)
            .whileHeld(IntakeCommand(self, -0.4))
        )

        # (Toggle) (Sider) (Back) Open/Close Compressor
        # (切换) (副操作手) (Back) 开/关 压缩机
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kBack)
            .toggleWhenPressed(CompressorCommand(self))
        )

        # (Hold) (Sider) (X) Conveyor Forward
        # (按住) (副操作手) (X) 传送带传球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kX)
            .whileHeld(ConveyorCommand(self, 0.3))
        )

        # (Hold) (Sider) (A) Conveyor Backward
        # (按住) (副操作手) (A) 传送带退球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kA)
            .whileHeld(ConveyorCommand(self, -0.3))
        )
