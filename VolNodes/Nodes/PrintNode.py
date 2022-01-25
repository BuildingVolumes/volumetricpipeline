from ryven.NENV import *
#from Nodes.TemplateNode import TemplateNode
class PrintNode(Node):
    """Generate a random number in a given range"""
    # this __doc__ string will be displayed as tooltip in the editor

    title = 'hogue'
    init_inputs = [
        NodeInputBP(),
    ]
    init_outputs = [
        NodeOutputBP(),
    ]
    color = '#aabb44'
    def __init__(self, params):
        super().__init__(params)
    def update_event(self, inp=-1):
        self.set_output_val(0, round(random.random() * self.input(0), 3))