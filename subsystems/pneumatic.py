from commands2 import SubsystemBase
from wpilib import Compressor, Solenoid, PneumaticsModuleType, SmartDashboard
import constants


class Pneumatic(SubsystemBase):

    def __init__(self):
        super().__init__()
        try:
            self.compressor = Compressor(PneumaticsModuleType.CTREPCM)
        except:
            print("Compressor Not Connected!")

        self.solenoidRight = Solenoid(
            PneumaticsModuleType.CTREPCM, constants.kSolenoidLeft)
        self.solenoidLeft = Solenoid(
            PneumaticsModuleType.CTREPCM, constants.kSolenoidRight)
        self.compressor.stop()
        self.prevSolenoidState = False

    def log(self):
        SmartDashboard.putData("Pneumatic", self)

    def periodic(self):
        self.log()

    def setCompressor(self, enable):
        if self.compressor is not None:
            if enable:
                self.compressor.start()
            else:
                self.compressor.stop()

    def set(self, enable):
        if enable != self.prevSolenoidState:
            self.solenoidLeft.set(enable)
            self.solenoidRight.set(not enable)

        self.prevSolenoidState = enable
