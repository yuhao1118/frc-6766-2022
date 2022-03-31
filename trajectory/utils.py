from wpimath.trajectory import Trajectory, TrajectoryConfig


def getInvertedTrajectory(trajectory: Trajectory) -> Trajectory:
    """
    Giving a trajectory that goes from A to B, it will return a 
    new trajectory that goes from B to A without turning round.
    """

    states = reversed(trajectory.states())

    for i, state in enumerate(states):

        _state = Trajectory.State(
            trajectory.states()[i].timeSeconds,
            state.velocityMetersPerSecond * -1.0,
            state.accelerationMetersPerSecondSq,
            state.poseMeters,
            state.curvatureRadPerMeter,
        )

        states[i] = _state

    return Trajectory(states)
