from ryvencore_qt.src.ryvencore import dtypes
from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
from BackEnd.randomValue import randValue

class StringNode(rc.Node):
    mainWindow = None
    data = None
    title = 'String'
    def __init__(self, params):
        super().__init__(params)
        self.shouldRun = False
        self.isStart = True

    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="String Value")
    ]

    init_outputs = [
        rc.NodeOutputBP()
    ]

    def update_event(self, inp=-1):
        print("running node " + self.title)
        self.set_output_val(0, self.input(0))