class LimelightTrackedTarget:
    def __init__(self, yaw, pitch, area, skew, pose, corners):
        self.yaw = yaw
        self.pitch = pitch
        self.area = area
        self.skew = skew
        self.pose = pose
        self.corners = corners

    def getArea(self):
        return self.area

    def getCameraRelativePose(self):
        return self.pose

    def getCorners(self):
        return self.corners

    def getPitch(self):
        return self.pitch

    def getSkew(self):
        return self.skew

    def getYaw(self):
        return self.yaw