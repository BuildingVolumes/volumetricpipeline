from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
import time
class SleepNode(TemplateNode):
    "A Debugging node that will do nothing for 10 seconds"
    title = 'Sleep'
    init_inputs = [
        rc.NodeInputBP()
    ]
    color = '#af3ecf'

    def __init__(self, params):
        super().__init__(params)
    def setupInputs(self):
        return self.input(0)
    def doStuff(self, inputData):
        time.sleep(10)
    def setupOutputs(self, outputData):
        pass