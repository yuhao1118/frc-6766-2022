from commands2 import SubsystemBase
from wpilib import SmartDashboard

import ctre
import constants


class Intaker(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.intaker = ctre.TalonFX(constants.kIntakePort)
        self.intaker.configFactoryDefault()
        self.intaker.setNeutralMode(ctre.NeutralMode.Brake)
        self.intaker.setInverted(constants.kIntakeRotate)

    def log(self):
        SmartDashboard.putData("Intaker", self)

    def periodic(self):
        # self.log()
        pass

    def set(self, output):
        self.intaker.set(ctre.ControlMode.PercentOutput, output)
