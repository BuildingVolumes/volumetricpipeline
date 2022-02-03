#from ryvencore_qt.src.WidgetBaseClasses import MWB
from ryvencore_qt import MWB
from ryven.NENV import *
import sys
import os
import json
from PySide2.QtWidgets import QFileDialog, QWidget, QVBoxLayout, QLabel, QPushButton, QLineEdit
from BackEnd.DirectorySaver import directorySaver
class ButtonNode(MWB, QWidget):
    def __init__(self, params):
        MWB.__init__(self, params)
        QWidget.__init__(self)
        self.setLayout(QVBoxLayout())
        
        b = QPushButton('click me')
        b.clicked.connect(self.button_clicked)
        self.layout().addWidget(b)
        textField = QLineEdit()
        textField.setReadOnly(True)
        textField.setText('None')
        self.layout().addWidget(textField)
        
        self.scriptFile = None

    def parseFile(self, filePath):
        f = open(filePath, "r")
        fileContents = f.read()

        jsonParsed = json.loads(fileContents)
        scriptName = jsonParsed["scripts"][0]["title"]
        return scriptName
    def setDir(self, data):
        self.scriptFile = data
    def button_clicked(self):
        dialog = QFileDialog()
        filePath = dialog.getOpenFileName(self, "Select  a File", ".", "Json (*.json)")[0]

        data = self.parseFile(filePath)
        print(data)
        if filePath == '':
            data = 'None'
        else:
            label = QLabel('another one')
            fileName = filePath.split('/')[-1]
            self.layout().itemAt(1).widget().setText(fileName)
            #self.layout().insertWidget(1, label)
            #print(self.layout().itemAt(1).text(data))
            self.title = "Macro Node" + ' (' + data + ')'
        self.set_state({'dir': data})
        
    def set_state(self, data: dict):
        self.setDir(data['dir'])
    def get_state(self) -> dict:
        data = {'dir': self.scriptFile}
        return data



class MacroNode(Node):
    title="Macro Node"
    main_widget_class = ButtonNode
    main_widget_pos = 'between ports'
    def __init__(self, params):
        super().__init__(params)
        self.subNodes = None
    
    def update_event(self, inp=-1):
        ...