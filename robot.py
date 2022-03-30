#!/usr/bin/env python3

import typing
import wpilib
import commands2
from wpiutil import PortForwarder

from robotcontainer import RobotContainer
import constants


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        try:
            PortForwarder.getInstance().add(5800, "10.67.66.30", 5800)
            # PortForwarder.add(5800, "limelight.local", 5800)
            # PortForwarder.add(5800, "photonvision.local", 5800)
        except Exception as e:
            print("Port Forwarder Not Connected!")
            print(repr(e))

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.container.robotDrive.setOpenloopRamp(
            constants.kOpenloopRampRateAuto)

        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        self.container.robotDrive.setOpenloopRamp(
            constants.kOpenloopRampRateTeleop)

        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
