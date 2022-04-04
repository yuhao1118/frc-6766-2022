import math

def clamp(value, minNum, maxNum):
    return max(minNum, min(value, maxNum))

def axisProfile(value, deadzone = 0.07, scaleFunction = lambda x: x ** 2):
    if abs(value) < deadzone:
        return 0.0

    return math.copysign(scaleFunction(value), value)
