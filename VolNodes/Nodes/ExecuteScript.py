from ryvencore_qt.src.ryvencore import dtypes
from ryvencore_qt.src.ryvencore.dtypes.dtypes import String
from Nodes.TemplateNode import TemplateNode
import ryvencore_qt as rc
import os
class ExecuteScriptNode(rc.Node):
    """Execute the code"""
    # all basic properties
    title = 'Execute Script'
    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Command"),
    ]

    color = '#A9D5EF'
    
    # see API doc for a full list of all properties

    # we could also skip the constructor here
    def __init__(self, params):
        super().__init__(params)
        #self.backendDir = ""
        # self.exeName = ""    
    def update_event(self, inp=-1):
        print('ran node ' + self.title)
        command = self.input(0)
        print(command)
        #finalCommand = self.backendDir + self.exeName + ' ' command
        #os.system(finalCommand)
        