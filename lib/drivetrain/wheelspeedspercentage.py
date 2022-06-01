from lib.utils.maths import clamp


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

    def __add__(self, otherWheelV):
        return WheelSpeedsPercentage(self.left + otherWheelV.left, self.right + otherWheelV.right)

    def __eq__(self, otherWheelV):
        return self.left == otherWheelV.left and self.right == otherWheelV.right

    def __mul__(self, scalar):
        return WheelSpeedsPercentage(self.left * scalar, self.right * scalar)
