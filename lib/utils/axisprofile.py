from wpimath.trajectory import TrapezoidProfile
import math


class AxisProfile:

    def __init__(self, deadzone):
        self.smoothFunction = lambda x: x * x
        self.deadzone = deadzone
        self.state = TrapezoidProfile.State()

    def setDeadzone(self, deadzone):
        self.deadzone = deadzone

    def setSmoothFunction(self, smoothFunction):
        self.smoothFunction = smoothFunction

    def reset(self, value):
        self.state = TrapezoidProfile.State(value, 0.0)

    def calculate(self, value):
        scaledValue = 0.0
        if abs(value) > self.deadzone:
            scaledValue = (abs(value) - self.deadzone) / (1 - self.deadzone)
            scaledValue = math.copysign(self.smoothFunction(scaledValue), value)

        profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(99999.0, 200.0),
            TrapezoidProfile.State(scaledValue, 0.0),
            self.state
        )

        self.state = profile.calculate(0.02)
        return self.state.position

