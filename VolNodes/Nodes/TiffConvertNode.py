from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
from BackEnd.tiffBgone import nodeConvertTiff
from threading import Thread
class TiffConvertNode(TemplateNode):
    "Recursively converts tiff files in a directory into PNG files"
    def __init__(self, params):
        super().__init__(params)
    title = 'Tiff Convert'
    init_inputs = [
        rc.NodeInputBP(label=' File Directory'),
        rc.NodeInputBP(label='Result File Type', dtype=rc.dtypes.String(default='png'), )
    ]
    init_outputs =[
        rc.NodeOutputBP()
    ]
    color = '#A9D5EF'

    def setupInputs(self):
        result = {'dir': self.input(0)+'/', 'fileType': self.input(1)}
        return result
    def doStuff(self, inputData):        
        nodeConvertTiff(inputData['dir'], inputData['dir'], inputData['fileType'], self.identifier)