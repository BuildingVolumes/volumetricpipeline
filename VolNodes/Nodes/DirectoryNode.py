from ryvencore_qt.src.WidgetBaseClasses import MWB
import ryvencore_qt as rc
from Nodes.TemplateNode import TemplateNode
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
        
        self.directory = None

        
    def setDir(self, data):
        self.directory = data
    def button_clicked(self):
        dialog = QFileDialog()
        data = dialog.getExistingDirectory(self, 'Select a Directory')
        if data == '':
            data = 'None'
        else:
            label = QLabel('another one')
            dataParts = data.split('/')
            
            self.layout().itemAt(1).widget().setText(".../"+dataParts[-1]+"/")
            #self.layout().insertWidget(1, label)
            #print(self.layout().itemAt(1).text(data))
            ...
        self.set_state({'dir': data})

    def set_state(self, data: dict):
        self.setDir(data['dir'])
    def get_state(self) -> dict:
        data = {'dir': self.directory}
        return data


class DirectoryNode(TemplateNode):
    "Gets a directory and sends the string of the directory to the next node"
    def __init__(self, params):
        super().__init__(params)
        self.isStart = True
    main_widget_class = ButtonNode
    title = 'Directory'
    init_inputs = []
    init_outputs = [
        rc.NodeOutputBP()
    ]
    color = '#fcba03'
    def setupInputs(self):
        data = self.main_widget().get_state()
        return data
        
    def doStuff(self, inputData):
        directorySaver(self.identifier, inputData['dir'])
        