from ryvencore_qt.src.ryvencore import dtypes
from ryvencore_qt.src.ryvencore.dtypes.dtypes import String
from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
class TextureOBJNode(rc.Node):
    """Prints your data"""
    # all basic properties
    title = 'Texture OBJ'
    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Preprend Command"),
        rc.NodeInputBP(dtype=rc.dtypes.String(), label=".obj File"),
        rc.NodeInputBP(label="Time"),
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="File Name"),
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="File Path")
    ]

    init_outputs = [
        rc.NodeOutputBP(label='Command Line')
    ]
    color = '#A9D5EF'
    
    # see API doc for a full list of all properties

    # we could also skip the constructor here
    def __init__(self, params):
        super().__init__(params)
    
    def update_event(self, inp=-1):
        print('ran node ' + self.title)
        command = self.input(0)
        command += " --TextureObj "
        for i in range(1, 4):
            command += self.input(i) + ' '

        print(command)
        