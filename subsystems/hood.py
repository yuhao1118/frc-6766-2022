from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController

import ctre
import constants


class Hood(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.hood = ctre.TalonFX(constants.kShooter)
        self.hood.configFactoryDefault()
        self.hood.setNeutralMode(ctre.NeutralMode.Coast)
        self.hood.setInverted(constants.kHoodRotate)

        self.hood.config_kP(0, constants.kPHood, 20)
        self.hood.config_kI(0, constants.kIHood, 20)
        self.hood.config_kD(0, constants.kDHood, 20)
        self.hood.configAllowableClosedloopError(0, 0.1 / constants.kHoodEncoderDegreesPerPulse, 20)

        self.resetActive = False
        self.resetComplete = False
        self.closedLoop = False

        self.resetGraceTimer = Timer()
        self.resetGraceTimer.start()


    def log(self):
        SmartDashboard.putData("Hood", self)
        SmartDashboard.putNumber("Hood Speed", self.getShooterEncoderSpeed())
        SmartDashboard.putNumber("Hood Position", value)

    def periodic(self):
        # self.log()
        self.hood.setNeutralMode(ctre.NeutralMode.Brake if DriverStation.getInstance().isEnabled() else ctre.NeutralMode.Coast)

        if DriverStation.getInstance().isDisabled():
            self.closedLoop = false

        if not self.resetComplete:
            if DriverStation.getInstance().isEnabled():
                if not self.resetActive:
                    self.resetActive = True
                    self.resetGraceTimer.reset()
                    self.setVolts(-0.03)
                else:
                    vel = self.getShooterEncoderSpeed()
                    if self.resetGraceTimer.hasElapsed(0.2) and abs(vel) < 0.1:
                        self.resetComplete = True
                        self.setVolts(0)
                        self.resetActive = False
                        self.resetGraceTimer.reset()
                        self.resetEncoder()
            else:
                self.resetActive = False

    def setVolts(self, outputVolts):
        self.hood.set(ctre.ControlMode.PercentOutput, outputVolts / 12)

    def setAngle(self, angle):
        position = angle / constants.kHoodEncoderDegreesPerPulse
        if self.resetComplete:
            self.hood.set(ctre.ControlMode.Position, angle)

    def resetEncoder(self):
        self.hood.setSelectedSensorPosition(0, 0, 20)

    def reset(self):
        self.resetComplete = False
        self.closedLoop = False
        
    def getShooterEncoderSpeed(self):
        return self.hood.getSelectedSensorVelocity() * constants.kHoodEncoderDegreesPerPulse * 10
    
    def getHoodEncoderPosition(self):
        return self.hood.getSelectedSensorPosition() * constants.kHoodEncoderDegreesPerPulse