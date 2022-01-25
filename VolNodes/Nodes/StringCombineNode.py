
import ryvencore_qt as rc
from Nodes.TemplateNode import TemplateNode
from BackEnd.CombineStrings import combineStrings

class StringCombineNode(TemplateNode):
    "Combines string inputs into an array of strings"
    def __init__(self, params):
        super().__init__(params)
        self.isStart = False
        self.numInputs = 2
        self.numUpdates = 0
        self.actions['add some input'] = {'method': self.add_inputs}
        self.actions['Remove last input'] = {'method': self.remove_input}
    title = 'String Combiner'
    init_inputs = [
        rc.NodeInputBP(label="String 1"),
        rc.NodeInputBP(label="String 2")
    ]
    init_outputs = [
        rc.NodeOutputBP(label='String Array'),
        rc.NodeOutputBP(label='Array Size')
    ]
    color = '#A9D5EF'
    def add_inputs(self):
        self.numInputs += 1
        self.create_input(label="String " + str(self.numInputs))
    
    def remove_input(self):
        if self.numInputs > 2:
            self.numInputs -= 1
            self.delete_input(self.numInputs)

    def setupInputs(self):
        stringArray = []
        for i in range(self.numInputs):
            stringArray.append(self.input(i))
        
        return stringArray
    def doStuff(self, inputData):
        
        combineStrings(self.identifier, inputData)
    
    def update_event(self, inp=-1):
        if self.shouldRun:
            self.numUpdates += 1
            if(self.numUpdates == self.numInputs):
                self.numUpdates = 0
                inputData = self.setupInputs()
                self.doStuff(inputData)
                outputData = self.loadNode(self.identifier)
                self.setupOutputs(outputData)