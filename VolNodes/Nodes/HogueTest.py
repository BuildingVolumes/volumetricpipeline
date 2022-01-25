from ryven.NENV import *


class Hogue(Node):
    """Rx,Ry,Rz"""
    # all basic properties
    title = 'RotXYZ'
    init_inputs = [
        NodeInputBP(dtype=dtypes.Float(1.0), label='rX'),
        NodeInputBP(dtype=dtypes.Float(1.0), label='rY'),
        NodeInputBP(dtype=dtypes.Float(1.0), label='rZ'),
        ]
    init_outputs = [
        NodeOutputBP(),
        NodeOutputBP(),
        NodeOutputBP(),
        ]
    color = '#A9D5EF'
    
    # see API doc for a full list of all properties

    # we could also skip the constructor here
    def __init__(self, params):
        super().__init__(params)
    
    def update_event(self, inp=-1):
        self.set_output_val(0, self.input(0))
        self.set_output_val(1, self.input(1))
        self.set_output_val(2, self.input(2))
        
