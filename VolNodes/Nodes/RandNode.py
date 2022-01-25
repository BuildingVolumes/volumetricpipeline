from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
from BackEnd.randomValue import randValue
from PySide2.QtCore import QObject, QThread, Signal
'''
class Worker(QObject):
    def __init__(self, node):
        super().__init__()
        self.node = node
    finished = Signal()
    progress = Signal()
    
    def run(self):
        inputData = self.node.setupInputs()
        self.node.doStuff(inputData)
        outputData = self.node.loadNode(self.node.identifier)
        
        self.node.setupOutputs(outputData)
        print('node thread finished')
        self.finished.emit()
'''
class RandNode(TemplateNode):
    """Generates random float"""
    def __init__(self, params):
        super().__init__(params)
        self.isStart = True

    title = 'Rand'
    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.Integer(default=1, bounds=(1, 100)))
    ]
    init_outputs = [
        rc.NodeOutputBP()
    ]
    color = '#fcba03'
    def view_place_event(self):
        print(self.mainWindow.updateProgress)
        #self.signals.notify_gui.connect(self.mainWindow.updateProgress)
    def setupInputs(self):
        return self.input(0)
    def doStuff(self, inputData):
        randValue(self.identifier, inputData)
            