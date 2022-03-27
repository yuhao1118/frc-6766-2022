from commands2 import Subsystem
from wpilib import Compressor, Solenoid, PneumaticsModuleType
import constants

class Pneumatic(Subsystem):

    def __init__(self):
        super().__init__()

        self.compressor = Compressor(PneumaticsModuleType.CTREPCM)
        self.solenoidRight = Solenoid(PneumaticsModuleType.CTREPCM, constants.kSolenoidLeft)
        self.solenoidLeft = Solenoid(PneumaticsModuleType.CTREPCM, constants.kSolenoidRight)
        self.compressor.stop()
        self.prevState = False

    def setCompressor(self, enable):
        if enable:
            self.compressor.start()
        else:
            self.compressor.stop()

    def set(self, enable):
        if enable != self.prevState:
            self.solenoidLeft.set(enable)
            self.solenoidRight.set(not enable)

        self.prevState = enable

