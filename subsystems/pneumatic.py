from commands2 import SubsystemBase
from wpilib import Compressor, Solenoid, PneumaticsModuleType, SmartDashboard
import constants


class Pneumatic(SubsystemBase):

    def __init__(self):
        super().__init__()
        try:
            self.compressor = Compressor(0, PneumaticsModuleType.CTREPCM)
            self.solenoidRight = Solenoid(
                0, PneumaticsModuleType.CTREPCM, constants.kSolenoidLeft)
            self.solenoidLeft = Solenoid(
                0, PneumaticsModuleType.CTREPCM, constants.kSolenoidRight)
            self.compressor.stop()
        except:
            print("Pneumatic connect error!")
            
        self.prevSolenoidState = False

    def log(self):
        SmartDashboard.putData("Pneumatic", self)

    def periodic(self):
        # self.log()
        pass

    def setCompressor(self, enable):
        if enable:
            self.compressor.start()
        else:
            self.compressor.stop()

    def set(self, enable):
        if enable != self.prevSolenoidState:
            self.solenoidLeft.set(enable)
            self.solenoidRight.set(not enable)

        self.prevSolenoidState = enable
