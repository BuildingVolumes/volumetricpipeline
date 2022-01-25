import ryvencore_qt as rc
import json
import os

class ManyInputNode(rc.Node):
    data = None
    mainWindow = None
    title = "Test"
    color = '#fcba03'
    numInputs = 1
    init_inputs = [
        #rc.NodeInputBP(dtype=rc.dtypes.String(), label="dir")
    ]

    def add_item(self, label, dataType):
        self.inputsIndex.append(label)
        self.create_input_dt(label=label, dtype=dataType, insert=self.numInputs)
        self.numInputs += 1
    
    def remove_item(self, label):
        firstOoccur = -1
        for i in range(len(self.inputsIndex)):
            if self.inputsIndex[i] == label:
                firstOoccur = i
        self.delete_input(firstOoccur)
        self.inputsIndex.remove(label)
        self.numInputs -= 1


    def enableAction(self, label, method):
        actionName = 'Add ' + label
        self.actions[actionName] = {'method': method}

        deletedName = 'Remove ' + label
        self.actions.pop(deletedName, None)
    
    def disableAction(self, label, method):

        deletedName = 'Add ' + label
        self.actions.pop(deletedName, None)

        actionName = 'Remove ' + label
        self.actions[actionName] = {'method': method}

    def __init__(self, params):
        super().__init__(params)
        self.shouldRun = False
        self.isStart = False
        self.inputsIndex = []
        #self.inputsIndex.append("dir")
        #self.actions["add String"] = {'method': self.add_string}
        #self.actions['Add Integer'] = {'method': self.add_int}

        self.numUpdates = 0
    def update_event(self, inp=-1):
            if self.shouldRun:
                print("running node " + self.title)
                self.numUpdates += 1
                if self.numUpdates == self.numInputs:
                    inputData = []
                    for i in range(len(self.inputIndex)):
                        inputData.append((self.inputIndex[i], self.input(i)))
                    print(inputData)
                    self.numUpdates = 0