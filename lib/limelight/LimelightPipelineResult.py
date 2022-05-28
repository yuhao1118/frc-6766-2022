class LimelightPipelineResult:
    def __init__(self,
                 latency,
                 target,
                 hasTargets
                 ):
        self._latency = latency
        self._bestTarget = target
        self._hasTargets = hasTargets

    def getBestTarget(self):
        return self._bestTarget

    def getCaptureTimestamp(self):
        return self._latency

    def getTargets(self):
        raise NotImplementedError

    def hasTargets(self):
        return self._hasTargets
