import ryvencore_qt as rc
import json
import os
class TemplateNode(rc.Node):
    data = None
    mainWindow = None
    def __init__(self,params):
        super().__init__(params)
        self.shouldRun = False
        self.deleteTempFiles = False
        self.isStart = False

    '''
    Setup Inputs should do the following things:
    Read data from the input sockets
    Format the data from the sockets in a way that the script in doStuff() is able to read
    '''
    def setupInputs():
        print('setupInputs should be overriden')
    '''
    setupOutputs should do the following things:
    Write the data from LoadNode to the sockets of the current node
    '''
    def setupOutputs(self, outputData):
        for key in outputData.keys():
            temp = int(key)
            self.set_output_val(temp, outputData[key])
    '''
    DoStuff should do the following things:
    Run code from the back end. This code can be anything (like a python script or a cpp file).
    Do stuff should pass the data it received into the Code (such as through Command Line Arguments)
    The code MUST output a json file that contains the variables.
    '''
    def doStuff():
        print('Do stuff should be overriden')
    '''
    loadNode will read the data from the JSON fule created in the doStuff() function and parse the data
    into a dictonary
    '''

    
    def loadNode(self, identifer):
        fileName = "./TempFiles/" + identifer + '.json'
        if not os.path.isfile(fileName):
            return None
        file = open(fileName, "r")
        
        jsonContent = file.read()
        file.close()
        if self.deleteTempFiles:
            if os.path.exists(fileName):
                os.remove(fileName)
            
        return json.loads(jsonContent) 
    def update_event(self, inp=-1):
        print(self.shouldRun)
        if self.shouldRun:
            
            inputData = self.setupInputs()
            self.doStuff(inputData)
            outputData = self.loadNode(self.identifier)
            
            self.setupOutputs(outputData)
            self.shouldRun = False
            self.signals.notify_gui.emit()
