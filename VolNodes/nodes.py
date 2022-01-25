from ryven.NENV import *

#this is important to ensure the path is set to the currect directory so it can find the other nodes
import sys
import os
sys.path.append(os.path.dirname(__file__))
from special_nodes import nodes as special_nodes

class NodeBase(Node):
    def __init__(self, params):
        super().__init__(params)

class Hogue_Node(NodeBase):
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

    def update_event(self, inp=-1):
        self.set_output_val(0, round(random.random() * self.input(0), 3))



export_nodes(
    *special_nodes,
    Hogue_Node,
)