### Code Reference ###
# https://github.com/SteelRidgeRobotics/2021-2022_FRC_Season/tree/main/Captain%20Hook

from commands2.button import JoystickButton, POVButton
from commands2 import ParallelCommandGroup
from wpilib import XboxController, SendableChooser, SmartDashboard

from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from lib.utils.tunablenumber import TunableNumber
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
from trajectory.trajectory import Trajectories

from commands.drivetrain.drive import DriveCommand
from commands.drivetrain.debugdrive import DebugDrive
from commands.drivetrain.debugautoaim import DebugAutoAim
from commands.drivetrain.autoaim import AutoAim
from commands.drivetrain.autoaimsimple import AutoAimSimple
from commands.shoot.resethood import ResetHoodCommandGroup
from commands.intake.intake import IntakeCommand
from commands.intake.compressor import CompressorCommand
from commands.intake.pneumatic import PneumaticCommand
from commands.conveyor.conveyor import ConveyorCommand
from commands.climb.elevator import ElevatorCommand
from commands.climb.arm import ArmCommand
from commands.climb.resetelevator import ResetElevatorCommand
from commands.autos.autopath import Auto1CommandGroup, Auto2CommandGroup, Auto3CommandGroup, TestCommandGroup
from commands.autos.autoshoot import PrepareShootCommandGroup, ManualShoot
from commands.autos.autoconvey import AutoConveyCommandGroup


class RobotContainer:

    def __init__(self):
        # 创建里程计实例
        # 创建手柄实例.
        self.driverController = XboxController(constants.kDriverControllerPort)
        self.siderController = XboxController(constants.kSiderControllerPort)

        # 创建各子系统实例.
        self.robotDrive = Drivetrain()
        self.odometry = self.robotDrive.getOdometry()
        self.elevatorDrive = Elevator()
        self.armDrive = Arm()
        self.flywheelDrive = Flywheel()
        self.hoodDrive = Hood()
        self.conveyorDrive = Conveyor()
        self.intakerDrive = Intaker()
        self.pneumaticControl = Pneumatic()
        self.visionControl = Vision()
        self.visionControl.setVisionOdometry(self.odometry)

        # 设置底盘默认指令, 允许机器人使用手柄控制.
        self.robotDrive.setDefaultCommand(
            DriveCommand(self, self.driverController)
        )

        # 在仪表盘显示自动阶段任务下拉选择器.
        self.autoCommandChooser = SendableChooser()  # 自动任务选择器
        self.autoCommandChooser.setDefaultOption("Auto1", Auto1CommandGroup(self))  # 3球自动
        self.autoCommandChooser.addOption("Auto2", Auto2CommandGroup(self))  # 2球自动
        self.autoCommandChooser.addOption("Auto3", Auto3CommandGroup(self))  # 2球自动 + 干扰敌方一球
        SmartDashboard.putData("Auto Command Chooser", self.autoCommandChooser)

        self.debugCommandChooser = SendableChooser()  # 调试指令选择器
        self.debugCommandChooser.setDefaultOption("DebugAutoAim", DebugAutoAim(self))
        self.debugCommandChooser.addOption("AutoAim", AutoAim(self, controller=self.driverController))
        self.debugCommandChooser.addOption("AutoAimSimple", AutoAimSimple(self, controller=self.driverController))
        self.debugCommandChooser.addOption("DebugDrive", DebugDrive(self))
        self.debugCommandChooser.addOption("DebugHood", HoodCommand(self, TunableNumber("Hood/debugAngle", 0.0)))
        self.debugCommandChooser.addOption("DebugFlywheel",
                                           FlywheelCommand(self, TunableNumber("Flywheel/debugRPS", 0.0)))
        self.debugCommandChooser.addOption("Test Forward", TestCommandGroup(self, Trajectories.ForwardTest))
        self.debugCommandChooser.addOption("Test Backward", TestCommandGroup(self, Trajectories.BackwardTest))
        self.debugCommandChooser.addOption("Test Auto11", TestCommandGroup(self, Trajectories.Auto11))
        self.debugCommandChooser.addOption("Test Auto12", TestCommandGroup(self, Trajectories.Auto12))
        self.debugCommandChooser.addOption("Test Auto2", TestCommandGroup(self, Trajectories.Auto2))
        self.debugCommandChooser.addOption("Test Auto31", TestCommandGroup(self, Trajectories.Auto31))
        self.debugCommandChooser.addOption("Test Auto32", TestCommandGroup(self, Trajectories.Auto32))
        SmartDashboard.putData("Debug Command Chooser", self.debugCommandChooser)

        # 设置手柄按键与对应指令的绑定.
        self.configureButtons()

    def getAutonomousCommand(self):
        return self.autoCommandChooser.getSelected()

    def getDebugCommand(self):
        return self.debugCommandChooser.getSelected()

    def getResetCommand(self):
        return ParallelCommandGroup(ResetHoodCommandGroup(self), ResetElevatorCommand(self)).withTimeout(2.0)

    def configureButtons(self):
        ############ 主操作手 ############
        # (按住) (主操作手) (LB+RB) 摄像头自瞄 (同时可以前后移动)
        (
            JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
                .and_(JoystickButton(self.driverController, XboxController.Button.kRightBumper))
                .whileActiveOnce(
                PrepareShootCommandGroup(self, aimMode="camera", controller=self.driverController).perpetually())
        )

        # (按住) (主操作手) (LB) 里程计自瞄 (同时可以前后移动)
        (
            JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
                .and_(JoystickButton(self.driverController, XboxController.Button.kRightBumper).not_())
                .whileActiveOnce(
                PrepareShootCommandGroup(self, aimMode="odometry", controller=self.driverController).perpetually())
        )

        # (按住) (主操作手) (B) 摇臂向车头
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kB)
                .whileHeld(ArmCommand(self, 0.15))
        )

        # (按住) (主操作手) (X) 摇臂向车尾
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kX)
                .whileHeld(ArmCommand(self, -0.15))
        )

        # (按住) (主操作手) (Y) 爬升, <缩>伸缩杆
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kY)
                .whileHeld(ElevatorCommand(self, 1.0))
        )

        # (按住) (主操作手) (A) 爬升, <升>伸缩杆
        (
            JoystickButton(self.driverController,
                           XboxController.Button.kA)
                .whileHeld(ElevatorCommand(self, -1.0))
        )

        if constants.tuningMode:
            # (切换) (主操作手) (start) 调试指令
            (
                JoystickButton(self.driverController,
                               XboxController.Button.kStart)
                    .toggleWhenPressed(self.getDebugCommand())
            )

        ############ 副操作手 ############
        # (按一下) (副操作手) (B) 射球
        (
            JoystickButton(self.siderController, XboxController.Button.kB)
                .whenPressed(ManualShoot(self))
        )

        # (按住) (副操作手) (LB) 开/关 Intake, 按住时打开Intake, 松手自动收起
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kLeftBumper)
                .whileHeld(PneumaticCommand(self, True))
        )

        # (按住) (副操作手) (Y) 吸球并传球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kY)
                .whileHeld(AutoConveyCommandGroup(self))
                .whenReleased(ConveyorCommand(self, -0.2).withTimeout(0.20))
        )

        # (按住) (副操作手) (A) 退球并吐球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kA)
                .whileHeld(AutoConveyCommandGroup(self, reverse=True))
        )

        # (切换) (副操作手) (start) 开/关 压缩机
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kStart)
                .toggleWhenPressed(CompressorCommand(self))
        )

        # (按住) (副操作手) (POV左) Intake吸球
        (
            POVButton(self.siderController,
                      POVEnum.kLeft)
                .whileHeld(IntakeCommand(self, 0.5))
        )

        # (按住) (副操作手) (POV右) Intake吐球
        (
            POVButton(self.siderController,
                      POVEnum.kRight)
                .whileHeld(IntakeCommand(self, -0.5))
        )

        # (按住) (副操作手) (POV上) 传送带传球
        (
            POVButton(self.siderController,
                      POVEnum.kUp)
                .whileHeld(ConveyorCommand(self, 0.5))
        )

        # (按住) (副操作手) (POV下) 传送带退球
        (
            POVButton(self.siderController,
                      POVEnum.kDown)
                .whileHeld(ConveyorCommand(self, -0.5))
        )
