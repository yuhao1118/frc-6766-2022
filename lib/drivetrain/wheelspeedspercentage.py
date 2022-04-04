from lib.utils.math import clamp

class WheelSpeedsPercentage:
    def __init__(self, left, right):
        self.left = clamp(left, -1.0, 1.0)
        self.right = clamp(right, -1.0, 1.0)

    def __str__(self):
        return "WheelSpeedsPercentage(left={}, right={})".format(self.left, self.right)

    def __repr__(self):
        return self.__str__()

    @classmethod
    def fromArcade(cls, throttle, turn):
        return WheelSpeedsPercentage(throttle + turn, throttle - turn)

    @classmethod
    def fromCurvature(cls, throttle, turn):
        turn = abs(throttle) * turn
        return WheelSpeedsPercentage(throttle + turn, throttle - turn)