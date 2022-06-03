from commands2 import SubsystemBase


class StatefulSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.defaultState = None
        self.curState = None
        self.lastState = None
        self.stateActionMap = {}

    def initStates(self):
        self.curState = self.defaultState
        self.getStateAction(self.defaultState).schedule()

    def addStateAction(self, state, action, default=False):
        if default:
            self.defaultState = state

        self.stateActionMap[state] = action

    def getStateAction(self, state):
        return self.stateActionMap.get(state)

    def hasChanged(self):
        if self.curState != self.lastState:
            self.lastState = self.curState
            return True
        return False

    def setState(self, state):
        self.curState = state

    def getState(self):
        return self.curState

