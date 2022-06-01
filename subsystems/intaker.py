from commands2 import SubsystemBase

import ctre
import constants


class Intaker(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.intaker = ctre.TalonFX(constants.kIntakeMotorPort)
        self.intaker.configFactoryDefault()
        self.intaker.setNeutralMode(ctre.NeutralMode.Brake)
        self.intaker.setInverted(constants.kIntakeRotate)

    def set(self, output):
        self.intaker.set(ctre.ControlMode.PercentOutput, output)
