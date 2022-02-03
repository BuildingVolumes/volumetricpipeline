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
import plyfile
from plyfile import PlyData, PlyElement
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


  
class GetDirname(Node):
    title = 'GetDirname'
    input_widget_classes = {
        'path input': widgets.PathInput
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

    def add_inp(self,theLabel=''):
        self.create_input(label=theLabel)

        index = self.num_inputs
        self.actions[f'remove input {index}'] = {
            'method': self.remove_inp,
            'data': index
        }

        self.num_inputs += 1

    def rename_inp(self, index, label):
        self.rename_input(index, label)
        
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
        self.code = str(self.input(0))
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

      
class CameraDirNode(_DynamicPorts_Node):
    title = 'CameraDirNode'
    
    init_inputs = [
        NodeInputBP(),
        NodeInputBP('index',dtype=dtypes.Integer(default=0, bounds=(1, 9999))),
    ]
    #init_outputs = [
    #    NodeOutputBP(label='Intrinsics'),
    #    NodeOutputBP(label='RGB'),
    #    NodeOutputBP(label='Depth'),
    #    NodeOutputBP(label='Matte'),
    #]
    
    def __init__(self, params):
        super().__init__(params)
        self.dir = ""
        self.index = 0
        self.dict = None
        super().add_out('Intrinsics')
        super().add_out('RGB')
        super().add_out('Depth')
        super().add_out('Matte')
        super().add_out('Dict')
        self.pin_intrin = 0
        self.pin_rgb = 1
        self.pin_depth = 2
        self.pin_matte = 3
        self.pin_dict = 4
        
    def resetPins(self) :
        for i in range(0,4,1) :
            super().set_output_val(i,None)
        
    def setOutputPinImageName(self, pinIndex, imName) :
        v = glob.glob(imName)
        print(v)
        if len(v) == 0 :
            return False
        else :
            value = v[0]
            super().set_output_val(pinIndex,value)      
            return True        
        
    def parseDir(self):
        #print("parse")
        #print(self.dir)
        #print(self.index)
        self.resetPins()
        pat = self.dir + "\\Intrinsics*.json"
        intrinMatch = glob.glob(pat)
        if len(intrinMatch) != 0 :
            intrin = intrinMatch[0]
        indS = str(self.index)
        rgbIm = self.dir + "\\Color_"+indS+".jpg"
        depthIm = self.dir + "\\Depth_"+indS+".tiff"
        matPat  = rgbIm + ".matte.png"
        matPat2  = self.dir + "\\Color_"+indS+".matte.png"
        self.setOutputPinImageName(self.pin_intrin, intrin)
        self.setOutputPinImageName(self.pin_rgb, rgbIm)
        self.setOutputPinImageName(self.pin_depth, depthIm)
        #sometimes I have .jpg.matte.png and other times I have .matte.png  ... uggh... FIX
        MATTE = matPat
        t = self.setOutputPinImageName(self.pin_matte, matPat)
        if t == False : 
            self.setOutputPinImageName(self.pin_matte,matPat2)
            MATTE = matPat2
        # setup dictionary pin
        self.dict = {'intrinsics':intrin, 'rgb':rgbIm, 'depth':depthIm, 'matte':MATTE, 'frame':self.index}
        super().set_output_val(self.pin_dict, self.dict)

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

    def __init__(self, img=None):
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
            
class VoxelCarveTSDF(_DynamicPorts_Node):
    title = 'VoxelCarveTSDF'
    version = 'v0.1'
    main_widget_class = widgets.ButtonNode_MainWidget
    main_widget_pos = 'between ports'
    # assume these are just paths to images and model

    init_inputs = [
     ]   
    init_outputs = [
        NodeOutputBP()
    ]
    def __init__(self, params):
        super().__init__(params)
        super().add_inp('Extrinsics')
        super().add_inp('OutputDir')
        super().add_inp('Cam0')
        self.frameNum = 0
        self.numCameras = 1
        self.lastPinIndex = 2
        self.firstCameraPinIndex = self.lastPinIndex
        self.outputDirName = "."
        self.main_exe = '.\\bin\\simpleTSDF.exe'
        self.command = self.main_exe
        
    def doVoxelCarveTSDF(self):
        print('doVoxelCarveTSDF')
        print('running command:'+self.command)
        os.system(self.command)
        
    def doButtonPress(self):
        print('doButtonPress - make command')
        self.command = self.main_exe
        extrinsics = self.input(0)
        self.command += ' -e '+  extrinsics
        self.outputDirName = self.input(1)
        for i in range(self.firstCameraPinIndex,self.lastPinIndex,1) :
            print('pin(index)='+str(i))
            print(self.input(i))
            cDict = self.input(i)
            intrin = cDict['intrinsics']
            rgb = cDict['rgb']
            depth = cDict['depth']
            matte = cDict['matte']
            self.frameNum = cDict['frame']
            outputPlyName = self.outputDirName+'\\output_'+str(self.frameNum)+'.ply'
            self.command += ' -i '+intrin 
            self.command += ' -r '+rgb
            self.command += ' -d '+depth
            self.command += ' -m '+matte
            self.command += ' -o '+outputPlyName
        self.doVoxelCarveTSDF()
        self.set_output_val(0, outputPlyName)
    
    def update_event(self, inp=-1):
        print('tsdf: update')
        print('inp:'+str(inp))
        if inp == self.lastPinIndex :
            label = "Cam"+str(inp-1)
            super().add_inp(label)
            self.numCameras = self.numCameras + 1
            self.lastPinIndex = self.lastPinIndex + 1
        if inp == -1 :
            self.doButtonPress()
            
        #ni = 0
        #if self.input(0) != None :
        #    self.cameraList = self.input(0)
        #    ni = ni+1
        #if self.input(1) != None :
        #    self.extrinsicsPath = self.input(1)
        #    ni = ni+1
        #
        #if ni == 2 :
        #    print('ready')
        #    self.doVoxelCarveTSDF()
            
     
class PlyfileRead(Node):
    title = 'Plyfile Read'
    version = 'v0.1'
    #main_widget_class = widgets.CustomGL_MESHWidget
    #main_widget_pos = 'between ports'
    # assume these are just paths to images and model
    init_inputs = [
        NodeInputBP('meshFile'),
    ]   
    init_outputs = [
        NodeOutputBP()
    ]
    def __init__(self, params):
        super().__init__(params)
        self.modelPath = None
        
        
    def update_event(self, inp=-1):
        if self.input(0) != None :
            self.modelPath = self.input(0)
            print('reading : '+self.modelPath)
            self.plydata = PlyData.read(self.modelPath)
            print('plyfile read')       
            print(self.plydata)
            self.set_output_val(0,self.plydata)         

class MeshIORead(Node):
    title = 'MeshIORead'
    version = 'v0.1'
    #main_widget_class = widgets.CustomGL_MESHWidget
    #main_widget_pos = 'between ports'
    # assume these are just paths to images and model
    init_inputs = [
        NodeInputBP('meshFile'),
    ]   
    init_outputs = [
        NodeOutputBP()
    ]
    def __init__(self, params):
        super().__init__(params)
        self.modelPath = None
        
    def update_event(self, inp=-1):
        if self.input(0) != None :
            self.modelPath = self.input(0)
            print('reading : '+self.modelPath)
            self.mesh = meshio.read(self.modelPath,)
            print('mesh read')       
            self.set_output_val(0,self.mesh)            
            
            

class GLMeshView(Node):
    """Prints your data"""
    # all basic properties
    title = 'GLMeshView'
    init_inputs = [
        NodeInputBP('mesh'),
    ]
    color = '#A9D5EF'
    main_widget_class = widgets.Custom_MESHWidget
	
    # see API doc for a full list of all properties

    # we could also skip the constructor here
    def __init__(self, params):
        super().__init__(params)
        self.mesh = None
        self.isChanged = False
    
    def update_event(self, inp=-1):
        if self.mesh != self.input(0) :
            self.mesh = self.input(0)
            self.isChanged = True
        self.update()



class ExtrinsicsLogParse(_DynamicPorts_Node):
    title = 'ExtrinsicsLogParse'
    #input_widget_classes = {
    #    'path input': widgets.FileInput
    #}
    init_inputs = [
        NodeInputBP('path'),
    ]

    def __init__(self, params):
        super().__init__(params)

        self.active = False
        self.dirpath = ''
        #self.actions['make executable'] = {'method': self.action_make_executable}
        super().clearout()

    def place_event(self):
        self.update()
        

    def readRow(self, file):
        line = file.readline()
        tokens = line.split()
        if len(tokens) != 4:
            print('error')
        r = np.array([float(tokens[0]), float(tokens[1]), float(tokens[2]), float(tokens[3])], dtype=float)
        return r
        
    def readMatrix(self,file):
        print('reading matrix')
        
        r1 = self.readRow(file)
        r2 = self.readRow(file)
        r3 = self.readRow(file)
        r4 = self.readRow(file)
        m = np.stack((r1,r2,r3,r4))
        return m
        
    def parseLog(self):
        print('parseLog')
        super().clearout()
        print('file:'+self.dirpath)
        file = open(self.dirpath,"r")
        #lines = file.readlines()
        count = 0
        self.dict = {}
        while True:
            # first line is always int int CAMERAID
            line = file.readline()
            if not line:
                break
            tokens = line.split()
            CAMERAID = int(tokens[2])
 
            # now grab the next 4 lines and parse them as a 4x4 matrix
            m = self.readMatrix(file)
            print(m)
            self.dict[CAMERAID] = m
            labelName = "c"+str(CAMERAID)
            super().add_out(labelName)
            super().set_output_val(count,m)
            count += 1
        file.close()
        print("found: "+str(count)+" transforms")
        super().add_out('Num')
        super().set_output_val(count, count)
        super().add_out('Dict')
        super().set_output_val(count+1,self.dict)
       
    def path_chosen(self, new_path):
        print('path chosen')
        self.dirpath = new_path
        self.parseLog()
        self.update()

    def update_event(self, inp=-1):
        print('update event')
        self.dirpath = self.input(0)
        self.parseLog()
        self.update()
        

    def get_state(self):
        print('get state - saving?')
        return { 
            **super().get_state(), 
            'path': self.dirpath,
        }

    def set_state(self, data, version):
        print('set state - loading?')
        ##super().set_state(data, version)
        #self.dirpath = data['path']
        #print('dirpath'+self.dirpath)
        #clientPat = self.dirpath + '/client_*'
        #print('clientpat:'+clientPat)
        #self.clients = glob.glob(clientPat)
        #print(self.clients)
        #self.numClients = data['numClients']
        #print(self.numClients)
        #self.extrinsicsLogName = data['extrin']
        #
        #i=0
        #for (clientname) in self.clients :
        #    theName = clientname
        #    tokens = clientname.split("client_")
        #    labelName = "c" + tokens[1]
        #    super().set_output_val(i,theName)
        #    i = i + 1
        #super().set_output_val(i,self.extrinsicsLogName)
        #print('done')

     
#GLViwer to rule them all
gptype = plyfile.PlyData([], text=True, obj_info=["test obj_info"])
class GLViewerDynamic(_DynamicPorts_Node):
    title = 'GLViewerALL'
    version = 'v0.1'
   # main_widget_class = widgets.ButtonNode_MainWidget
   # main_widget_pos = 'between ports'
    # assume these are just paths to images and model

    init_inputs = [
     ]   
    
    def __init__(self, params):
        super().__init__(params)
        self.default_label = "INPUT................."
        super().add_inp(self.default_label)
        self.lastPinIndex = 0
        # types we can handle
        self.stringType = ""
        self.intType = 0
        self.floatType = 1.5
        testim = np.ones((1,1,1),np.uint8)*255
        self.plyfileType = gptype
        self.imageType = CVImage(testim)
        
        
    def doButtonPress(self):
        print('doButtonPress - make command')
    
    def HandleInput(self, theInput) :
        label = ""
        print(type(theInput))
        print(type(self.imageType))
        if type(theInput) is type(self.stringType) :
            label = "string"
        elif type(theInput) is type(self.intType) :
            label = "int"
        elif type(theInput) is type(self.floatType) :
            label = "float"
        elif type(theInput) is type(self.imageType) :
            label = "image"
        elif type(theInput) is type(self.plyfileType) :
            label = "plyfile"
        return label
    
    def update_event(self, inp=-1):
        if inp == self.lastPinIndex :
            # check input type
            # handle based on input type
            label = self.HandleInput(self.input(inp))
            stype = str(type(self.input(inp)))
            super().rename_inp(inp, label)
            super().add_inp(self.default_label)
            self.lastPinIndex = self.lastPinIndex + 1
        if inp == -1 :
            self.doButtonPress()
            
        #ni = 0
        #if self.input(0) != None :
        #    self.cameraList = self.input(0)
        #    ni = ni+1
        #if self.input(1) != None :
        #    self.extrinsicsPath = self.input(1)
        #    ni = ni+1
        #
        #if ni == 2 :
        #    print('ready')
        #    self.doVoxelCarveTSDF()
            

nodes = [
    GLNode,
    Button_Node,
    Livescan3dDir,
    Show_Node,
    CameraDirNode,
    DisplayImg,
    ReadImage,
    MatteExtractor,
    GetFilename, GetDirname, 
    VoxelCarveTSDF,
    MeshIORead,
    GLMeshView,
    PlyfileRead,
    ExtrinsicsLogParse,
    GLViewerDynamic,
]
