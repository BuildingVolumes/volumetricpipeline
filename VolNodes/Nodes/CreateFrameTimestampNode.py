from Nodes.ManyInputNode import ManyInputNode

import ryvencore_qt as rc

class CreateFrameTimestampNode(ManyInputNode):
    title = "Frame from Timestamp"
    color = '#fcba03'
    numInputs = 2

    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Camera Manager"),
        #rc.NodeInputBP(dtype=rc.dtypes.Integer(), label="Timestamp")
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Voxel Grid Data")
    ]

    def __init__(self, params):
        super().__init__(params)
        self.inputsIndex.append("Camera Manager")
        self.inputsIndex.append("Voxel Grid Data")
        self.actions["Add Timestamp"] = {'method': self.add_timestamp}
        self.actions["Remove Timestamp"] = {'method': self.remove_timestamp}

    def add_timestamp(self):
        self.add_item("Timestamp", rc.dtypes.Integer())

    def remove_timestamp(self):
        if "Timestamp" in self.inputsIndex:
            self.remove_item("Timestamp")
