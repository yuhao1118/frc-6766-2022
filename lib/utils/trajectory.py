from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryUtil
from commands2 import CommandBase
import os

class DebugCommand(CommandBase):
    def __init__(self, kinematics, trajectory):
        super().__init__()
        self.m_kinematics = kinematics
        self.m_trajectory = trajectory
        self.m_timer = Timer()

    def initialize(self):
        self.m_timer.reset()
        self.m_timer.start()

    def execute(self):
        state = self.m_trajectory.sample(self.m_timer.get())
        targetWheelSpeeds = self.m_kinematics.toWheelSpeeds(
            ChassisSpeeds(
                state.velocity,
                state.velocity * state.curvature,
            )
        )

        leftSpeedSetpoint = targetWheelSpeeds.left
        rightSpeedSetpoint = targetWheelSpeeds.right

        SmartDashboard.putNumber("Left Speed Setpoint", leftSpeedSetpoint)
        SmartDashboard.putNumber("Right Speed Setpoint", rightSpeedSetpoint)
    
    def isFinished(self):
        return False


def getInvertedTrajectory(trajectory: Trajectory) -> Trajectory:
    """
    Giving a trajectory that goes from A to B, it will return a 
    new trajectory that goes from B to A without turning round.
    """

    states = reversed(trajectory.states())

    for i, state in enumerate(states):

        _state = Trajectory.State(
            trajectory.states()[i].timeSeconds,
            state.velocity * -1.0,
            state.acceleration,
            state.pose,
            state.curvature,
        )

        states[i] = _state

    return Trajectory(states)

def getTrajectoryFromFile(filename, deployPath="/home/lvuser/py/deploy/pathplanner/generatedJSON/"):
    trajectoryPath = os.path.join(deployPath, filename)
    return TrajectoryUtil.fromPathweaverJson(trajectoryPath)
