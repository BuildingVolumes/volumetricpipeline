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
        rc.NodeInputBP(label=' Camera Manager'),
        rc.NodeInputBP(label='Voxel Grid Data', dtype=rc.dtypes.String())
    ]
    init_outputs =[
        rc.NodeOutputBP()
    ]
    color = '#A9D5EF'

    def setupInputs(self):
        result = {}
        return result
    def doStuff(self, inputData):        
        nodeConvertTiff(inputData['dir'], inputData['dir'], inputData['fileType'], self.identifier)
    
    def loadNode(self, identifer):
        return identifer
    
    def setupOutputs(self, outputData):
        ...
        
        

        