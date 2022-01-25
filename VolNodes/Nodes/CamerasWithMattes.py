import ryvencore_qt as rc

class LoadCamerasLSNode(rc.Node):
    data = None
    mainWindow = None
    title = "Load Cameras LiveScan Mattes"

    init_inputs = [
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Prepend String"),
        rc.NodeInputBP(dtype=rc.dtypes.String(), label="Root Folder"),
        rc.NodeInputBP(dtype=rc.dtypes.Float(), label="Playback Speed")
    ]
    
    init_outputs = [
        rc.NodeOutputBP('Command line')
    ]

    def update_event(self, inp=-1):
        command = self.input(0)
        command += " --LoadCamerasLivescanWithMattes "
        command += self.input(1) + ' ' + self.input(2)

        self.set_output_val(0, command)
        print('ran node ' + self.title)