#!/usr/bin/env python3

import wpilib
import commands2

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand = None
    container = None

    def robotInit(self):
        self.container = RobotContainer()

    def disabledInit(self):
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.container.getAutonomousCommand().schedule()
        self.container.getResetCommand().schedule()
        self.container.getStateMachineCommand().schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
