from networktables import NetworkTables
from pyfrc.physics.visionsim import VisionSim, VisionSimTarget


class LimelightSim(VisionSim):

    def __init__(self,
                 targets,
                 camera_fov,
                 view_dst_start,
                 view_dst_end,
                 data_frequency=15,
                 data_log=0.050,
                 physics_controller=None
                 ):
        """
        Note: Simulation works for yaw axis only! 
        There are a lot of constructor parameters:

        :param targets:          List of target positions (x, y) on field in meter
        :param camera_fov:       Field of view of camera (in degrees)
        :param view_dst_start:   If the robot is closer than this, the target cannot be seen
        :param view_dst_end:     If the robot is farther than this, the target cannot be seen
        :param data_frequency:   How often the camera transmits new coordinates
        :param data_log:         How long it takes for the camera data to be processed
                                 and make it to the robot
        :param physics_controller: If set, will draw target information in UI
        """

        super().__init__(targets, camera_fov, view_dst_start, view_dst_end,
                         data_frequency=data_frequency, data_lag=data_log, physics_controller=physics_controller)

        self.visionTable = NetworkTables.getTable('limelight')
        self.visionTable.putNumber('tv', 0)
        self.visionTable.putNumber('tx', 0)
        self.visionTable.putNumber('ty', 0)
        self.visionTable.putNumber('ts', 0)
        self.visionTable.putNumber('ta', 5)

    def update(self, now, robot_pose):
        currentTranslation = robot_pose.translation()
        currentRotation = robot_pose.rotation()

        x = currentTranslation.X()
        y = currentTranslation.Y()
        angle = currentRotation.radians()

        data = self.compute(now, x, y, angle)
        if data is not None:
            has_target, capture_time, angle, distance = data[0]
            self.visionTable.putNumber('tv', has_target)
            self.visionTable.putNumber('tx', -angle if angle != float("inf") else 0.0)
