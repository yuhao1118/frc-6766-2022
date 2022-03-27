from commands2 import SubsystemBase

import ctre
import constants


class Conveyor(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.conveyor = ctre.TalonFX(constants.kConveyorPort)
        self.conveyor.configFactoryDefault()
        self.conveyor.setNeutralMode(ctre.NeutralMode.Brake)
        self.conveyor.setInverted(constants.kConveyorRotate)

    def set(self, output):
        self.conveyor.set(ctre.ControlMode.PercentOutput, output)
