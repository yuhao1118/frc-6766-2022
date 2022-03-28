from commands2 import SubsystemBase
from wpilib import SmartDashboard

import ctre
import constants


class Conveyor(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.conveyor = ctre.TalonFX(constants.kConveyorPort)
        self.conveyor.configFactoryDefault()
        self.conveyor.setNeutralMode(ctre.NeutralMode.Brake)
        self.conveyor.setInverted(constants.kConveyorRotate)

    def log(self):
        SmartDashboard.putData("Conveyor", self)

    def periodic(self):
        self.log()

    def set(self, output):
        self.conveyor.set(ctre.ControlMode.PercentOutput, output)
