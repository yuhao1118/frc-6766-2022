import ctre
from lib.utils.math import clamp
from lib.enums.pov import POVEnum
import constants

class DifferentialDrive:
    """
    Teleop-drive functions for TalonFX based drivetrain
    """
    def __init__(self, LF_motor, RF_motor):
        self.LF_motor = LF_motor
        self.RF_motor = RF_motor

    def tankDrive(self, leftPercentage, rightPercentage):
        self.LF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, leftPercentage)
        self.RF_motor.set(
            ctre.TalonFXControlMode.PercentOutput, rightPercentage)

    def tankDriveVolts(self, leftVolts, rightVolts):
        self.tankDrive(leftVolts / 12, rightVolts / 12)

    def arcadeDrive(self, throttle, turn, smoothInputs=True):
        if smoothInputs:
            if abs(throttle) < 0.07:
                throttle = 0
            
            if abs(turn) < 0.07:
                turn = 0

            turn = turn ** 3 * constants.kDrivetrainTurnSensitive
            throttle = throttle ** 3

        leftSpeed = clamp(throttle + turn, -1.0, 1.0)
        rightSpeed = clamp(throttle - turn, -1.0, 1.0)

        self.tankDrive(leftSpeed, rightSpeed)

    def povDrive(self, povButton):
        # Using POV button to adjust the drivetrain

        if povButton == POVEnum.kUp:
            self.arcadeDrive(0.2, 0, smoothInputs=False)
        elif povButton == POVEnum.kDown:
            self.arcadeDrive(-0.2, 0, smoothInputs=False)
        elif povButton == POVEnum.kRight:
            self.arcadeDrive(0, 0.2, smoothInputs=False)
        elif povButton == POVEnum.kLeft:
            self.arcadeDrive(0, -0.2, smoothInputs=False)