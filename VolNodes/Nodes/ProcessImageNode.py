from Nodes.ManyInputNode import ManyInputNode
import ryvencore_qt as rc
class ProcessImageNode(ManyInputNode):
    title = "Process Images"
    color = '#fcba03'
    numInputs = 1
    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Directory")
    ]

    def __init__(self, params):
        super().__init__(params)
        self.inputsIndex.append("Directory")
        self.actions["Add Model"] = {'method': self.add_model}
        self.actions['Add BG Dir'] = {'method': self.add_bg_dir}
    
    def add_model(self):
        self.add_item("Model", rc.dtypes.String())
        

        self.disableAction("Model", self.remove_model)
    def remove_model(self):
        self.remove_item("Model")

        self.enableAction("Model", self.add_model)

    def add_bg_dir(self):
        self.add_item("BG Dir", rc.dtypes.String())
        
        self.disableAction("BG Dir", self.remove_bg_dir)

    def remove_bg_dir(self):
        self.remove_item("BG Dir")

        self.enableAction("BG Dir", self.add_bg_dir)