#!/usr/bin/env python3

import wpilib
import commands2

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    container = None

    def robotInit(self):
        wpilib.LiveWindow.disableAllTelemetry()
        self.container = RobotContainer()

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.container.getAutonomousCommand().schedule()
        self.container.getResetCommand().schedule()
        self.container.getStateMachineCommand().schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

        self.container.getResetCommand().schedule()
        self.container.getStateMachineCommand().schedule()

    def teleopPeriodic(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
