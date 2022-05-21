from wpilib import SmartDashboard
import constants

class TunableNumber():
    def __init__(self, name, defaultValue=0.0):
        self.key = "TunableNumbers/" + name
        self.defaultValue = defaultValue
        self.lastHasChangedValue = defaultValue
        self.setDefault(defaultValue)

    def get(self):
        return SmartDashboard.getNumber(self.key, self.defaultValue) if constants.tuningMode else self.defaultValue

    def setDefault(self, defaultValue):
        self.defaultValue = defaultValue
        if constants.tuningMode:
            SmartDashboard.putNumber(self.key, SmartDashboard.getNumber(self.key, defaultValue))
        else:
            SmartDashboard.delete(key)

    def getDefault(self):
        return self.defaultValue

    def hasChanged(self):
        currentValue = self.get()
        if currentValue != self.lastHasChangedValue:
            self.lastHasChangedValue = currentValue
            return True
        return False

    def __float__(self):
        return self.get()
