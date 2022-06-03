### Code Reference ###
# https://github.com/SteelRidgeRobotics/2021-2022_FRC_Season/tree/main/Captain%20Hook

from commands2 import ParallelCommandGroup
from wpilib import SendableChooser, SmartDashboard

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
from ios.singleControllerIO import SingleControllerIO
from ios.dualControllerIO import DualControllerIO
from lib.utils.tunablenumber import TunableNumber
from trajectory.trajectory import Trajectories

from commands.shoot.flywheel import FlywheelCommand
from commands.shoot.hood import HoodCommand
from commands.drivetrain.drive import DriveCommand
from commands.drivetrain.debugdrive import DebugDrive
from commands.drivetrain.turntoangle import TurnToAngle
from commands.drivetrain.autoaim import AutoAim
from commands.drivetrain.autoaimsimple import AutoAimSimple
from commands.shoot.resethood import ResetHoodCommandGroup
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
        # 创建IO实例.
        self.io = SingleControllerIO(controllerPort=constants.kDriverControllerPort)

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
        self.robotDrive.setDefaultCommand(DriveCommand(self, self.io))

        # 在仪表盘显示自动阶段任务下拉选择器.
        self.autoCommandChooser = SendableChooser()  # 自动任务选择器
        self.autoCommandChooser.setDefaultOption("Auto1", Auto1CommandGroup(self))  # 3球自动
        self.autoCommandChooser.addOption("Auto2", Auto2CommandGroup(self))  # 2球自动
        self.autoCommandChooser.addOption("Auto3", Auto3CommandGroup(self))  # 2球自动 + 干扰敌方一球
        SmartDashboard.putData("Auto Command Chooser", self.autoCommandChooser)

        self.debugCommandDict = {
            "DebugAutoAim": TurnToAngle(self),
            "AutoAim": AutoAim(self, shouldAutoTerminate=True),
            "AutoAimSimple": AutoAimSimple(self, shouldAutoTerminate=True),
            "DebugDrive": DebugDrive(self),
            "DebugHood": HoodCommand(self, TunableNumber("Hood/debugAngle", 0.0)),
            "DebugFlywheel": FlywheelCommand(self, TunableNumber("Flywheel/debugRPS", 0.0)),
            "TestForward": TestCommandGroup(self, Trajectories.ForwardTest),
            "TestBackward": TestCommandGroup(self, Trajectories.BackwardTest),
            "TestAuto11": TestCommandGroup(self, Trajectories.Auto11),
            "TestAuto12": TestCommandGroup(self, Trajectories.Auto12),
            "TestAuto2": TestCommandGroup(self, Trajectories.Auto2),
            "TestAuto31": TestCommandGroup(self, Trajectories.Auto31),
            "TestAuto32": TestCommandGroup(self, Trajectories.Auto32),
        }  # 调试指令列表

        # 设置手柄按键与对应指令的绑定.
        self.configureButtons()

    def getAutonomousCommand(self):
        return self.autoCommandChooser.getSelected()

    def getResetCommand(self):
        return ParallelCommandGroup(ResetHoodCommandGroup(self), ResetElevatorCommand(self)).withTimeout(2.0)

    def getStateMachineCommand(self):
        return ParallelCommandGroup(
            ElevatorCommand(self, self.io)
        )

    def configureButtons(self):
        # (按住) 摄像头自瞄 (同时可以前后移动)
        self.io.getSimpleAimButton().whileActiveOnce(PrepareShootCommandGroup(self,
                                                                              aimMode="camera",
                                                                              io=self.io,
                                                                              shouldAutoTerminate=False))
        # (按住) 里程计自瞄 (同时可以前后移动)
        self.io.getGlobalAimButton().whileActiveOnce(PrepareShootCommandGroup(self,
                                                                              aimMode="odometry",
                                                                              io=self.io,
                                                                              shouldAutoTerminate=False))
        # (按住) 摇臂向车头
        self.io.getClimbArmForwardButton().whileActiveOnce(ArmCommand(self, 0.15).perpetually())
        # (按住) 摇臂向车尾
        self.io.getClimbArmBackwardButton().whileActiveOnce(ArmCommand(self, -0.15).perpetually())
        # (切换) 调试指令
        if constants.tuningMode:
            self.io.getDebugButton().toggleWhenActive(self.debugCommandDict["DebugAutoAim"])
        # (按一下) 射球
        self.io.getShootButton().whenActive(ManualShoot(self))
        # (按住) 开/关 Intake, 按住时打开Intake, 松手自动收起
        self.io.getIntakerButton().whileActiveOnce(PneumaticCommand(self, True).perpetually())
        # (按住) 吸球并传球
        self.io.getCargoIntakeButton().whileActiveOnce(AutoConveyCommandGroup(self).perpetually()
                                                       ).whenInactive(ConveyorCommand(self, -0.2).withTimeout(0.20))
        # (按住) 退球并吐球
        self.io.getCargoOuttakeButton().whileActiveOnce(AutoConveyCommandGroup(self, reverse=True))
        # (切换) 开/关 压缩机
        self.io.getCompressorButton().toggleWhenActive(CompressorCommand(self))
