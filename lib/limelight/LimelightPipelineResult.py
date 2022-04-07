class LimelightPipelineResult:
    def __init__(self,
        latency,
        target,
        hasTargets
    ):
        self.latency = latency
        self.bestTarget = target
        self.hasTargets = hasTargets

    def getBestTarget(self):
        return self.bestTarget

    def getLatency(self):
        return self.latency

    def getTargets(self):
        raise NotImplementedError

    def hasTargets(self):
        return self.hasTargets

    