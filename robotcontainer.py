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
from lib.enums.pov import POVEnum
from trajectory.trajectory import Trajectory
from commands import pneumaticcommand, compressorcommand, intakecommand, conveyorcommand, drivecommand, shootercommand, climbarmcommand, climbcommand, softlimitscommand
from commands.autos.getcellsandshoot import IntakeConveyCommandGroup, AutoShootCommandGroup
from commands.autos.getrangeandaim import GetRangeAndAimCommand
from commands.autos.autopath import Auto1CommandGroup, Auto2CommandGroup, TestCommandGroup


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
        self.climberDrive = Climber()
        self.shooterDrive = Shooter()
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
            drivecommand.DriveCommand(self, self.driverController)
        )

        # Display the autonomous chooser on the SmartDashboard.
        # 在仪表盘显示自动阶段任务下拉选择器.
        self.autoChooser = SendableChooser()
        
        self.autoChooser.setDefaultOption(
            "Auto1", Auto1CommandGroup(self))                               # 3 ball auto. 3球自动
        self.autoChooser.addOption(
            "Auto2", Auto2CommandGroup(self))                               # 2 ball auto. 2球自动

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
            "Test Auto21",  TestCommandGroup(self, Trajectory.Auto2))

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
        SmartDashboard.putData("Shooter Forward High", shootercommand.ShooterCommand(self, output=shootercommand.shooterSpeedHigh['0cm']))
        SmartDashboard.putData("Shooter Forward Low", shootercommand.ShooterCommand(self, output=shootercommand.shooterSpeedLow['0cm']))
        SmartDashboard.putData("Shooter Backward", shootercommand.ShooterCommand(self, output=-5.0))



    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()

    def configureButtons(self):
        """Configure the buttons for the driver's controller"""

        ############ Auto-assisted control / 自动辅助控制 ############

        # (Hold) (Sider) (LB) Open/Close Intake
        # (按住) (副操作手) (LB) 开/关 Intake, 按住时打开Intake, 松手自动收起
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kLeftBumper)
            .whileHeld(pneumaticcommand.PneumaticCommand(self, True))
        )

        # (Hold) (Sider) (RB) Intake and convey
        # (按住) (副操作手) (RB) 吸球并传球
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kRightBumper)
            .whileHeld(IntakeConveyCommandGroup(self))
        )

        # (Hold) (Sider) (Start) Backball and drop
        # (按住) (副操作手) (Start) 退球并吐球
        (
            JoystickButton(self.siderController,
                            XboxController.Button.kStart)
            .whileHeld(IntakeConveyCommandGroup(self, reverse=True))
        )

        # (Press) (Sider) (Y) Aiming
        # (按一下) (副操作手) (Y) 自瞄
        (
            JoystickButton(self.siderController, XboxController.Button.kY)
            .whenPressed(GetRangeAndAimCommand(self).withTimeout(0.8))
        )

        # (Press) (Sider) (B) Shooting - fixed distance at 0cm
        # (按一下) (副操作手) (B) 射球 - 0cm固定距离
        (
            JoystickButton(self.siderController, XboxController.Button.kB)
            .whenPressed(AutoShootCommandGroup(self))
        )

        ############ Manual Controls ############

        # (Hold) (Driver) (B) Arm Forward
        # (按住) (主操作手) (B) 爬升摇臂向前
        (
            JoystickButton(self.driverController,
                      XboxController.Button.kB)
            .whileHeld(climbarmcommand.ClimbArmCommand(self, 0.1))
        )

        # (Hold) (Driver) (X) Arm Backward
        # (按住) (主操作手) (X) 爬升摇臂向后
        (
            JoystickButton(self.driverController,
                        XboxController.Button.kX)
            .whileHeld(climbarmcommand.ClimbArmCommand(self, -0.1))
        )

        # (Hold) (Driver) (LB) Climb Up
        # (按住) (主操作手) (LB) 爬升, <缩>伸缩杆
        (
            JoystickButton(self.driverController,
                        XboxController.Button.kLeftBumper)
            .whileHeld(climbcommand.ClimbCommand(self, 0.6))
        )

        # (Hold) (Driver) (RB) Climb Down
        # (按住) (主操作手) (RB) 爬升, <升>伸缩杆
        (
            JoystickButton(self.driverController,
                        XboxController.Button.kRightBumper)
            .whileHeld(climbcommand.ClimbCommand(self, -0.8))
        )

        # (Toggle) (Driver) (Back) enable/disable soft limits
        # (切换) (主操作手) (Back) 启用 / 禁用软限位, 第一次按下时为禁用(解除)软限位
        (
            JoystickButton(self.driverController,
                        XboxController.Button.kBack)
            .toggleWhenPressed(softlimitscommand.SoftLimitsCommand(self))
        )

        # (Hold) (Sider) (POV-Up) Intake Motor Forward
        # (按住) (副操作手) (POV上) Intake吸球
        (
            POVButton(self.siderController,
                      POVEnum.kUp)
            .whileHeld(intakecommand.IntakeCommand(self, 0.4))
        )

        # (Hold) (Sider) (POV-Down) Intake Motor Backward
        # (按住) (副操作手) (POV下) Intake吐球
        (
            POVButton(self.siderController,
                      POVEnum.kDown)
            .whileHeld(intakecommand.IntakeCommand(self, -0.4))
        )

        # (Toggle) (Sider) (Back) Open/Close Compressor
        # (切换) (副操作手) (Back) 开/关 压缩机
        (
            JoystickButton(self.siderController,
                           XboxController.Button.kBack)
            .toggleWhenPressed(compressorcommand.CompressorCommand(self, True))
        )

        # (Hold) (Sider) (X) Conveyor Forward
        # (按住) (副操作手) (X) 传送带传球
        (
            JoystickButton(self.siderController,
                      XboxController.Button.kX)
            .whileHeld(conveyorcommand.ConveyorCommand(self, 0.3))
        )

        # (Hold) (Sider) (A) Conveyor Backward
        # (按住) (副操作手) (A) 传送带退球
        (
            JoystickButton(self.siderController,
                      XboxController.Button.kA)
            .whileHeld(conveyorcommand.ConveyorCommand(self, -0.3))
        )
