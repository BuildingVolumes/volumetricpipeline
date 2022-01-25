from Nodes.TemplateNode import TemplateNode
from ryven.NENV import *
import code
from contextlib import redirect_stdout, redirect_stderr

#this is important to ensure the path is set to the currect directory so it can find the other nodes
import sys
import os
from os import walk
import glob

# open cv 
import cv2
import numpy as np

sys.path.append(os.path.dirname(__file__))

from Nodes.HogueTest import Hogue
widgets = import_widgets(__file__)


  
class GetFilename(Node):
    title = 'GetFilename'
    input_widget_classes = {
        'path input': widgets.FileInput
    }
    init_inputs = [
        NodeInputBP('path', add_data={'widget name': 'path input', 'widget pos': 'below'}),
    ]
    init_outputs = [
        NodeInputBP(),
    ]

    def __init__(self, params):
        super().__init__(params)
        self.active = False
        self.filepath = ''
        self.actions['make executable'] = {'method': self.action_make_executable}
        
    def view_place_event(self):
        self.input_widget(0).path_chosen.connect(self.path_chosen)

        
    def path_chosen(self, new_path):
        print('path chosen')
        self.filepath = new_path
        self.update()

    def action_make_executable(self):
        self.create_input(type_='exec', insert=0)
        self.active = True

        del self.actions['make executable']
        self.actions['make passive'] = {'method': self.action_make_passive}

    def action_make_passive(self):
        self.delete_input(0)
        self.active = False

        del self.actions['make passive']
        self.actions['make executable'] = {'method': self.action_make_executable}

    def update_event(self, inp=-1):
        print('update event')
        self.set_output_val(0,self.filepath)
        
    def get_state(self):
        print('get state')
        return { 
            **super().get_state(), 
            'path': self.filepath 
        }

    def set_state(self, data, version):
        print('set state')
        self.filepath = data['path']
        self.set_output_val(0,self.filepath)




class Print_Node(Node):
    title = 'Button'
    version = 'v0.1'
    main_widget_class = widgets.ButtonNode_MainWidget
    main_widget_pos = 'between ports'
    init_inputs = [

    ]
    init_outputs = [
        NodeOutputBP()
    ]
    color = '#99dd55'
    def __init__(self, params):
        super().__init__(params)
        self.value = False
        
    def update_event(self, inp=-1):
        if self.value == False :
            self.value = True
        else : 
            self.value = False
        self.set_output_val(0,self.value)


class Button_Node(Node):
    title = 'Button'
    version = 'v0.1'
    main_widget_class = widgets.ButtonNode_MainWidget
    main_widget_pos = 'between ports'
    init_inputs = [

    ]
    init_outputs = [
        NodeOutputBP()
    ]
    color = '#99dd55'
    def __init__(self, params):
        super().__init__(params)
        self.value = False
        
    def update_event(self, inp=-1):
        if self.value == False :
            self.value = True
        else : 
            self.value = False
        self.set_output_val(0,self.value)

class GLNode(Node):
    """Prints your data"""
    # all basic properties
    title = 'OpenGL View'
    init_inputs = [
        NodeInputBP(dtype=dtypes.Float(1.0), label='rX'),
        NodeInputBP(dtype=dtypes.Float(1.0), label='rY'),
        NodeInputBP(dtype=dtypes.Float(1.0), label='rZ'),
        NodeInputBP(dtype=dtypes.Boolean(False), label='anim'),
    ]
    color = '#A9D5EF'
    main_widget_class = widgets.Custom_MainWidget
	
    # see API doc for a full list of all properties

    # we could also skip the constructor here
    def __init__(self, params):
        super().__init__(params)
        self.rX = 1.0
        self.rY = 1.0
        self.rZ = 1.0
        self.animating = False
    
    def update_event(self, inp=-1):
        self.rX = self.input(0)
        self.rY = self.input(1)
        self.rZ = self.input(2)
        if self.input(3) == True :
            self.animating = True
        else :
            self.animating = False

        self.update()




class _DynamicPorts_Node(Node):
    version = 'v0.1'
    init_inputs = []
    init_outputs = []

    def __init__(self, params):
        super().__init__(params)

        self.actions['add input'] = {'method': self.add_inp}
        self.actions['add output'] = {'method': self.add_out}

        self.num_inputs = 0
        self.num_outputs = 0

    def add_inp(self):
        self.create_input()

        index = self.num_inputs
        self.actions[f'remove input {index}'] = {
            'method': self.remove_inp,
            'data': index
        }

        self.num_inputs += 1

    def remove_inp(self, index):
        self.delete_input(index)
        self.num_inputs -= 1
        del self.actions[f'remove input {self.num_inputs}']

    def add_out(self, theLabel=''):
        self.create_output(label=theLabel)

        index = self.num_outputs
        self.actions[f'remove output {index}'] = {
            'method': self.remove_out,
            'data': index
        }

        self.num_outputs += 1

    def clearout(self):
        for i in range(0,self.num_outputs,1) :
            remove_out(i)
        
        
    def remove_out(self, index):
        self.delete_output(index)
        self.num_outputs -= 1
        del self.actions[f'remove output {self.num_outputs}']

    def get_state(self) -> dict:
        return {
            'num inputs': self.num_inputs,
            'num outputs': self.num_outputs,
        }

    def set_state(self, data: dict):
        print(data)
        self.num_inputs = data['num inputs']
        self.num_outputs = data['num outputs']
        print('dyn: num outputs:'+self.num_outputs)



class Show_Node(_DynamicPorts_Node):
    title = 'ShowNode'
    version = 'v0.1'
    main_widget_class = widgets.CodeNode_MainWidget
    main_widget_pos = 'between ports'
    init_inputs = [
        NodeInputBP(),
    ]
    def __init__(self, params):
        super().__init__(params)
       
        self.code = ""

    def place_event(self):
        pass

    def update_event(self, inp=-1):
        self.code = self.input(0)
        #self.main_widget().update_text(self.node.code)
        print('update:' + self.code)
        if self.session.gui and self.main_widget():
            self.main_widget().update_text(self.code)

    def get_state(self) -> dict:
        return {
            **super().get_state(),
            'code': self.code,
        }

    def set_state(self, data: dict, version):
        super().set_state(data, version)
        self.code = self.input(0)
        print('set_state:' + self.code)

    
    
    
class Livescan3dDir(_DynamicPorts_Node):
    title = 'Livescan3d Dir'
    input_widget_classes = {
        'path input': widgets.PathInput
    }
    init_inputs = [
        NodeInputBP('path', add_data={'widget name': 'path input', 'widget pos': 'below'}),
    ]

    def __init__(self, params):
        super().__init__(params)

        self.active = False
        self.dirpath = ''
        self.actions['make executable'] = {'method': self.action_make_executable}
        super().clearout()

    def place_event(self):
        self.update()
        
    def view_place_event(self):
        self.input_widget(0).path_chosen.connect(self.path_chosen)

    def parseDir(self):
        print('parseDir')
        super().clearout()
        clientPat = self.dirpath + '/client_*'
        self.clients = glob.glob(clientPat)
        self.numClients = len(self.clients)
        extrinsicsPat = self.dirpath + '/Extrinsics*.log'
        self.extrinsicsLogName = glob.glob(extrinsicsPat)[0]
        
        print(self.clients)
        print(self.numClients)
        i=0
        for (clientname) in self.clients :
            theName = clientname
            tokens = clientname.split("client_")
            labelName = "c" + tokens[1]
            super().add_out(labelName)
            super().set_output_val(i,theName)
            i = i + 1
        
        super().add_out('Extrin')
        super().set_output_val(i,self.extrinsicsLogName)
        
    def path_chosen(self, new_path):
        print('path chosen')
        self.dirpath = new_path
        self.parseDir()
        self.update()

    def action_make_executable(self):
        self.create_input(type_='exec', insert=0)
        self.active = True

        del self.actions['make executable']
        self.actions['make passive'] = {'method': self.action_make_passive}

    def action_make_passive(self):
        self.delete_input(0)
        self.active = False

        del self.actions['make passive']
        self.actions['make executable'] = {'method': self.action_make_executable}

    def update_event(self, inp=-1):
        print('update event')
        #i=0
        #for (clientname) in self.clients :
        #    self.set_output_val(i, clientname)
        #    i = i + 1
        #self.set_output_val(i, self.extrinsicsLogName)

    def get_state(self):
        print('get state - saving?')
        return { 
            **super().get_state(), 
            'path': self.dirpath,
            'extrin': self.extrinsicsLogName,
            'numClients':self.numClients
        }

    def set_state(self, data, version):
        print('set state - loading?')
        #super().set_state(data, version)
        self.dirpath = data['path']
        print('dirpath'+self.dirpath)
        clientPat = self.dirpath + '/client_*'
        print('clientpat:'+clientPat)
        self.clients = glob.glob(clientPat)
        print(self.clients)
        self.numClients = data['numClients']
        print(self.numClients)
        self.extrinsicsLogName = data['extrin']

        i=0
        for (clientname) in self.clients :
            theName = clientname
            tokens = clientname.split("client_")
            labelName = "c" + tokens[1]
            super().set_output_val(i,theName)
            i = i + 1
        super().set_output_val(i,self.extrinsicsLogName)
        print('done')

      
class CameraDirNode(Node):
    title = 'CameraDirNode'
    
    init_inputs = [
        NodeInputBP(),
        NodeInputBP('index',dtype=dtypes.Integer(default=0)),
    ]
    init_outputs = [
        NodeOutputBP(label='Intrinsics'),
        NodeOutputBP(label='RGB'),
        NodeOutputBP(label='Depth'),
    ]
    
    def __init__(self, params):
        super().__init__(params)
        self.dir = ""
        self.index = 0

    def parseDir(self):
        print("parse")
        print(self.dir)
        print(self.index)
        pat = self.dir + "\\Intrinsics*.json"
        intrin = glob.glob(pat)[0]
        indS = str(self.index)
        rgbIm = self.dir + "\\Color_"+indS+".jpg"
        depthIm = self.dir + "\\Depth_"+indS+".tiff"

        
        print(intrin)
        print(rgbIm)
        print(depthIm)
        print('-----------')
        self.set_output_val(0,intrin)
        self.set_output_val(1,rgbIm)
        self.set_output_val(2,depthIm)
        

    def update_event(self, inp=-1):
        print('update')
        self.dir = self.input(0)
        self.index = self.input(1)
        self.parseDir()
        

#-----START OPENCV STUFF -----------------------------------

class CVImage:
    """
    The OpenCV Mat(rix) data type seems to have overridden comparison operations to perform element-wise comparisons
    which breaks ryverncore-internal object comparisons.
    To avoid this, I'll simply use this wrapper class and recreate a new object every time for now, so ryvencore
    doesn't think two different images are the same.
    """

    def __init__(self, img):
        self.img = img
        self.dtype = np.dtype('uint8')


class ReadImage(Node):
    """Reads an image from a file"""
    title = 'Read Image'
    init_inputs = [
        NodeInputBP(),
        NodeInputBP('isDepth', dtype=dtypes.Boolean(False)),
    ]
    init_outputs = [
        NodeOutputBP('img')
    ]

    def __init__(self, params):
        super().__init__(params)
        self.image_filepath = ''
        self.isDepth = False

    def view_place_event(self):
        self.image_filepath = self.input(0)
        self.isDepth = self.input(1)
        self.update()
        #self.input_widget(0).path_chosen.connect(self.path_chosen)
        # self.main_widget_message.connect(self.main_widget().show_path)

    def update_event(self, inp=-1):
        self.image_filepath = self.input(0)
        self.isDepth = self.input(1)
        if self.image_filepath == '':
            return
        try:
            if self.isDepth : 
                theImage = CVImage(cv2.imread(self.image_filepath, cv2.IMREAD_ANYDEPTH))
                theImage.dtype = np.dtype('uint16')
                print(theImage.dtype)
                self.set_output_val(0, theImage)
                
            else : 
                theImage =  CVImage(cv2.imread(self.image_filepath, cv2.IMREAD_ANYDEPTH|cv2.IMREAD_ANYCOLOR))
                theImage.dtype = np.dtype('uint8')
                print(theImage.dtype)
                self.set_output_val(0,theImage)
        except Exception as e:
            print(e)

    def get_state(self):
        data = {'image file path': self.image_filepath}
        return data

    def set_state(self, data, version):
        #self.path_chosen(data['image file path'])
        self.image_filepath = self.input(0)




class OpenCVNodeBase(Node):
    init_outputs = [
        NodeOutputBP()
    ]
    main_widget_class = widgets.OpenCVNode_MainWidget
    main_widget_pos = 'below ports'

    def __init__(self, params):
        super().__init__(params)

        if self.session.gui:
            from qtpy.QtCore import QObject, Signal
            class Signals(QObject):
                new_img = Signal(object)

            # to send images to main_widget in gui mode
            self.SIGNALS = Signals()

    def view_place_event(self):
        self.SIGNALS.new_img.connect(self.main_widget().show_image)

        try:
            self.SIGNALS.new_img.emit(self.get_img())
        except:  # there might not be an image ready yet
            pass

    def update_event(self, inp=-1):
        new_img_wrp = CVImage(self.get_img())

        if self.session.gui:
            self.SIGNALS.new_img.emit(new_img_wrp.img)

        self.set_output_val(0, new_img_wrp)

    def get_img(self):
        return None


class DisplayImg(OpenCVNodeBase):
    title = 'Display Image'
    init_inputs = [
        NodeInputBP('img'),
    ]   
    def get_img(self):
        return self.input(0).img

#-----END OPENCV STUFF -----------------------------------

#-------MATTE EXTRACTOR NODE---------------
import torch
from torchvision.transforms.functional import to_tensor, to_pil_image
from PIL import Image
class MatteExtractor(Node):
    title = 'MatteExtractor'
    version = 'v0.1'
    #main_widget_class = widgets.ButtonNode_MainWidget
    #main_widget_pos = 'between ports'
    # assume these are just paths to images and model
    init_inputs = [
        NodeInputBP('ref'),
        NodeInputBP('img'),
        NodeInputBP('model'),
    ]   
    init_outputs = [
        NodeOutputBP()
    ]
    def __init__(self, params):
        super().__init__(params)
        
    def doMatteExtract(self):
        print('matte extract')
        src = Image.open(self.imgPath)
        bgr = Image.open(self.refPath)
        src = to_tensor(src).cuda().unsqueeze(0)
        bgr = to_tensor(bgr).cuda().unsqueeze(0)
        self.theModel.backbone_scale = 1/4
        self.theModel.refine_sample_pixels = 80_000
        pha, fgr = self.theModel(src, bgr)[:2]
        self.outputFileName = self.imgPath + ".matte.png"
        to_pil_image(pha[0].cpu()).save(self.outputFileName)
        self.set_output_val(0,self.outputFileName)
        
    def update_event(self, inp=-1):
        ni = 0
        if self.input(0) != None :
            self.refPath = self.input(0)
            ni = ni+1
        if self.input(1) != None :
            self.imgPath = self.input(1)
            ni = ni+1
        if self.input(2) != None :
            self.modelPath = self.input(2)
            self.theModel = torch.jit.load(self.modelPath).cuda().eval()
            ni = ni+1
        
        if ni == 3 :
            print('ready')
            self.doMatteExtract()
            
    
nodes = [
    GLNode,
    Button_Node,
    Livescan3dDir,
    Show_Node,
    CameraDirNode,
    DisplayImg,
    ReadImage,
    MatteExtractor,
    GetFilename,
]
