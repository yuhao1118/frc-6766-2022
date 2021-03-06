from commands2 import SubsystemBase
from wpilib import Compressor, Solenoid, PneumaticsModuleType, reportError
import constants


class Pneumatic(SubsystemBase):

    def __init__(self):
        super().__init__()
        self.isPneumaticInit = False
        try:
            self.compressor = Compressor(PneumaticsModuleType.CTREPCM)
            self.solenoidRight = Solenoid(PneumaticsModuleType.CTREPCM, constants.kSolenoidLeft)
            self.solenoidLeft = Solenoid(PneumaticsModuleType.CTREPCM, constants.kSolenoidRight)
            # self.compressor.disable()
            self.setAutoMode()
            self.isPneumaticInit = True
        except:
            reportError("Pneumatic subsystem init failed")
            
        self.prevSolenoidState = False

    def setAutoMode(self):
        self.compressor.enableDigital()

    def setCompressor(self, enable):
        if self.isPneumaticInit:
            if enable:
                self.compressor.enableDigital()
            else:
                self.compressor.disable()

    def set(self, enable):
        if self.isPneumaticInit:
            if enable != self.prevSolenoidState:
                self.solenoidLeft.set(enable)
                self.solenoidRight.set(not enable)

            self.prevSolenoidState = enable
