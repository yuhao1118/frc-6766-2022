#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from wpilib import RobotController, ADXRS450_Gyro
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import constants
from lib.sensors.wit_imu import WitIMUSim
from pyfrc.physics.core import PhysicsInterface
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class TalonFXMotorSim:
    def __init__(self, motor, kvVoltSecondsPerMeter, kDistancePerPulse) -> None:
        self.simCollection = motor.getSimCollection()
        self.kvVoltSecondsPerMeter = kvVoltSecondsPerMeter
        self.kDistancePerPulse = kDistancePerPulse


    def update(self, dt) -> None:
        voltage = self.simCollection.getMotorOutputLeadVoltage()
        velocity = voltage / self.kvVoltSecondsPerMeter

        self.simCollection.setIntegratedSensorVelocity(
            int(velocity / self.kDistancePerPulse / 10)
        )
        self.simCollection.addIntegratedSensorPosition(
            int(velocity * dt / self.kDistancePerPulse)
        )

    def getVoltage(self):
        return float(self.simCollection.getMotorOutputLeadVoltage())



class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):

        self.physics_controller = physics_controller

        self.LF_motor = TalonFXMotorSim(
            robot.container.robotDrive.LF_motor, 
            constants.kvVoltSecondsPerMeter, 
            constants.kDrivetrainEncoderDistancePerPulse)
        self.LR_motor = TalonFXMotorSim(
            robot.container.robotDrive.LR_motor, 
            constants.kvVoltSecondsPerMeter, 
            constants.kDrivetrainEncoderDistancePerPulse)
        self.RF_motor = TalonFXMotorSim(
            robot.container.robotDrive.RF_motor, 
            constants.kvVoltSecondsPerMeter, 
            constants.kDrivetrainEncoderDistancePerPulse)
        self.RR_motor = TalonFXMotorSim(
            robot.container.robotDrive.RR_motor, 
            constants.kvVoltSecondsPerMeter, 
            constants.kDrivetrainEncoderDistancePerPulse)

        self.system = LinearSystemId.identifyDrivetrainSystem(
            constants.kvVoltSecondsPerMeter,
            0.2,
            2.5,  # The angular velocity gain, in volt seconds per angle.
            0.3,  # The angular acceleration gain, in volt seconds^2 per angle.
        )

        self.drivesim = DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidthMeters,
            DCMotor.falcon500(constants.kDrivetrainMotorCount),
            constants.kDrivetrainGearRatio,
            (constants.kDrivetrainWheelDiameterMeters / 2),
        )

        self.gyro = WitIMUSim(robot.container.robotDrive.gyro)

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.gyro.setAngle(self.drivesim.getHeading().degrees())

        self.LF_motor.update(tm_diff)
        self.LR_motor.update(tm_diff)
        self.RF_motor.update(tm_diff)
        self.RR_motor.update(tm_diff)
        
        self.drivesim.setInputs(-self.LF_motor.getVoltage(), self.RF_motor.getVoltage())
        self.drivesim.update(tm_diff)

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())