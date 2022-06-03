from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation, Timer

import ctre
import constants
from lib.utils.tunablenumber import TunableNumber


class Hood(SubsystemBase):

    def __init__(self):
        super().__init__()

        self.hood = ctre.TalonFX(constants.kHoodMotorPort)
        self.hood.configFactoryDefault()
        self.hood.setNeutralMode(ctre.NeutralMode.Coast)
        self.hood.setInverted(constants.kHoodRotate)

        self.hood.configForwardSoftLimitThreshold(constants.kHoodMotorSoftLimitForward, 0)

        self.kP = TunableNumber("Hood/kP", 0.60)
        self.kI = TunableNumber("Hood/kI", 0.00)
        self.kD = TunableNumber("Hood/kD", 45.00)
        self.kF = TunableNumber("Hood/kF", 0.00)

        self.hood.config_kP(0, self.kP.getDefault(), 0)
        self.hood.config_kI(0, self.kI.getDefault(), 0)
        self.hood.config_kD(0, self.kD.getDefault(), 0)
        self.hood.config_kF(0, self.kF.getDefault(), 0)
        self.hood.configAllowableClosedloopError(
            0, int(0.33 / constants.kHoodEncoderDegreesPerPulse), 0)
        self.hood.configClosedLoopPeakOutput(0, 0.3, 0)
        self.hood.configClosedloopRamp(0.2)

        self.resetActive = False
        self.resetComplete = False
        self.closedLoop = False
        self.resetGraceTimer = Timer()
        self.resetGraceTimer.start()

        self.goalPosition = 0.0

        self.setActive = False
        self.hoodReady = False

    def log(self):
        SmartDashboard.putData("Hood", self)
        SmartDashboard.putNumber(
            "Hood Output", self.hood.getMotorOutputPercent())
        SmartDashboard.putNumber("Hood Speed", self.getHoodEncoderSpeed())
        SmartDashboard.putNumber(
            "Hood Position", self.getHoodEncoderPosition())

    def periodic(self):
        # self.log()
        if self.kP.hasChanged():
            self.hood.config_kP(0, float(self.kP), 0)
        if self.kI.hasChanged():
            self.hood.config_kI(0, float(self.kI), 0)
        if self.kD.hasChanged():
            self.hood.config_kD(0, float(self.kD), 0)
        if self.kF.hasChanged():
            self.hood.config_kF(0, float(self.kF), 0)

        self.hood.setNeutralMode(ctre.NeutralMode.Brake if DriverStation.getInstance(
        ).isEnabled() else ctre.NeutralMode.Coast)
        self.closedLoop = DriverStation.getInstance().isEnabled()

        if not self.resetComplete:
            self.goalPosition = 0.0
            if DriverStation.getInstance().isEnabled():
                if not self.resetActive:
                    self.resetActive = True
                    self.resetGraceTimer.reset()
                    self.set(-0.08)
                else:
                    vel = self.getHoodEncoderSpeed()
                    if self.resetGraceTimer.hasElapsed(0.2) and abs(vel) < 0.1:
                        self.resetComplete = True
                        self.set(0)
                        self.resetActive = False
                        self.resetGraceTimer.reset()
                        self.resetEncoder()
            else:
                self.resetActive = False

        if self.closedLoop and self.resetComplete:
            self.hood.set(ctre.ControlMode.Position, self.goalPosition)

            if self.setActive and abs(self.hood.getClosedLoopError()) < \
                    int(0.33 / constants.kHoodEncoderDegreesPerPulse):
                self.hoodReady = True

    def set(self, output):
        self.hood.set(ctre.ControlMode.PercentOutput, output)

    def setAngle(self, angle):
        self.goalPosition = angle / constants.kHoodEncoderDegreesPerPulse
        self.setActive = True

    def moveToBottom(self):
        self.setAngle(0.0)

    def resetEncoder(self):
        self.hood.setSelectedSensorPosition(0, 0, 0)
        self.hood.configForwardSoftLimitEnable(True, 0)

    def reset(self):
        self.hood.configForwardSoftLimitEnable(False, 0)
        self.resetComplete = False
        self.closedLoop = False
        self.setActive = False

    def isHoodReady(self):
        return self.hoodReady

    def getHoodEncoderSpeed(self):
        # deg / s
        return self.hood.getSelectedSensorVelocity() * constants.kHoodEncoderDegreesPerPulse * 10

    def getHoodEncoderPosition(self):
        # deg
        return self.hood.getSelectedSensorPosition() * constants.kHoodEncoderDegreesPerPulse
