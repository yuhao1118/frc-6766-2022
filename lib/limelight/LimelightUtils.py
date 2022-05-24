import math


def estimateDistance(cameraPitchRad,
                     targetPitchRad,
                     cameraHeight,
                     targetHeight,
                     cameraYawRad):
    return (targetHeight - cameraHeight) / math.tan(cameraPitchRad + targetPitchRad) / math.cos(cameraYawRad)
