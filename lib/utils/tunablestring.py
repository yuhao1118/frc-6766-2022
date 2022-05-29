from wpilib import SmartDashboard
import constants


class TunableString():
    def __init__(self, name, defaultValue=""):
        self.key = "TunableStrings/" + name
        self.defaultValue = defaultValue
        self.lastHasChangedValue = defaultValue
        self.setDefault(defaultValue)

    def get(self):
        return SmartDashboard.getString(self.key, self.defaultValue) if constants.tuningMode else self.defaultValue

    def setDefault(self, defaultValue):
        self.defaultValue = defaultValue
        if constants.tuningMode:
            SmartDashboard.putString(self.key, SmartDashboard.getString(self.key, defaultValue))
        else:
            SmartDashboard.delete(self.key)

    def getDefault(self):
        return self.defaultValue

    def hasChanged(self):
        currentValue = self.get()
        if currentValue != self.lastHasChangedValue:
            self.lastHasChangedValue = currentValue
            return True
        return False

    def __str__(self):
        return self.get()
