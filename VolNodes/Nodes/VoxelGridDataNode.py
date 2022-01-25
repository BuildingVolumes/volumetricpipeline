import ryvencore_qt as rc
from torch._C import dtype

class VoxelGridDataNode(rc.Node):
    data = None
    mainWindow = None
    title = "Voxel Grid"

    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(default=""), label="Prepend String"),
        rc.NodeInputBP(dtype=rc.dtypes.Integer(default=1000, bounds=(1, 10000)), label="Blocks"),
        rc.NodeInputBP(dtype=rc.dtypes.Float(default=1000.0), label="Depth Scale"),
        rc.NodeInputBP(dtype=rc.dtypes.Float(default=3.0), label="Depth Scale"),
        rc.NodeInputBP(dtype=rc.dtypes.String(default="CPU:0"), label="Device Code"),
        rc.NodeInputBP(dtype=rc.dtypes.Float(default=0.04), label="SDF Truc"),
        rc.NodeInputBP(dtype=rc.dtypes.Float(default=0.005859375), label="Voxel Size")

    ]
    
    init_outputs = [
        rc.NodeOutputBP('Command line')
    ]

    def update_event(self, inp=-1):
        command = self.input(0, 6)
        command += " --EditVoxelGridData "
        for i in range(1, ):
            command += self.input(i) + ' '

        self.set_output_val(0, command)
        print('ran node ' + self.title)