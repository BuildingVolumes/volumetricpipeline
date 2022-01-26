import re

from ryven.NENV import *
from PySide2.QtWidgets import QLabel, QPushButton, QFileDialog, QVBoxLayout, QWidget, QTextEdit, QScrollArea, QOpenGLWidget, QComboBox, QPlainTextEdit, QLineEdit
from PySide2.QtGui import QImage, QPixmap, QFont
from PySide2.QtCore import Signal, QSize, QTimer, QEvent
from PySide2 import QtOpenGL
from qtpy.QtWidgets import QSlider
from OpenGL.GL import *
from OpenGL.arrays import vbo
from OpenGL import GLU

import numpy as np



import cv2
import os


class ButtonNode_MainWidget(QPushButton, MWB):

    def __init__(self, params):
        MWB.__init__(self, params)
        QPushButton.__init__(self)

        self.clicked.connect(self.update_node)



class Open_MainWidget(MWB, QLabel):
    def __init__(self, params):
        MWB.__init__(self, params)
        QLabel.__init__(self)

        self.resize(200, 200)

    def show_image(self, img):
        self.resize(200, 200)
        print('show_image')
        print('blah blah')
        try:
            if img.dtype == uint16 :
                print('main widget: 16 bit')
                im2 = (img/256).astype('uint8')
                rgb_image = cv2.cvtColor(im2, cv2.COLOR_GRAY2RGB)
            else :
                print('main widget: 8 bit')
                rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except cv2.error:
            return

        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        img_w = qt_image.width()
        img_h = qt_image.height()
        proportion = img_w / img_h
        self.resize(self.width() * proportion, self.height())
        qt_image = qt_image.scaled(self.width(), self.height())
        self.setPixmap(QPixmap(qt_image))
        self.node.update_shape()

class Custom_CodeWidget(QTextEdit):
    def __init__(self):
        QTextEdit.__init__(self)

        self.setFont(QFont('Consolas', 9))
        self.setPlainText('import cv2\nimg = None')
        self.setFixedHeight(100)
        self.setMinimumWidth(300)


class CustomGL_CodeWidget(QOpenGLWidget):
    def __init__(self):
        QOpenGLWidget.__init__(self)
        self.setFixedHeight(400)
        self.setMinimumWidth(400)
        
        
        
    def initGeometry(self):
        self.cubeVtxArray = np.array(
            [[0.0, 0.0, 0.0],
             [1.0, 0.0, 0.0],
             [1.0, 1.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, 1.0],
             [1.0, 0.0, 1.0],
             [1.0, 1.0, 1.0],
             [0.0, 1.0, 1.0]])
        self.vertVBO = vbo.VBO(np.reshape(self.cubeVtxArray, (1, -1)).astype(np.float32))
        self.vertVBO.bind()
        self.cubeClrArray = np.array(
            [[0.0, 0.0, 0.0],
             [1.0, 0.0, 0.0],
             [1.0, 1.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, 1.0],
             [1.0, 0.0, 1.0],
             [1.0, 1.0, 1.0],
             [0.0, 1.0, 1.0 ]])
        self.colorVBO = vbo.VBO(np.reshape(self.cubeClrArray, (1, -1)).astype(np.float32))
        self.colorVBO.bind()
        self.cubeIdxArray = np.array(
            [0, 1, 2, 3,
             3, 2, 6, 7,
             1, 0, 4, 5,
             2, 1, 5, 6,
             0, 3, 7, 4,
             7, 6, 5, 4 ])
             
    def setRotX(self, val):
        if val == None : 
            self.rotX = 0
        else :
            self.rotX = val

    def setRotY(self, val):
        if val == None : 
            self.rotY = 0
        else :
            self.rotY = val
    def setRotZ(self, val):
        if val == None : 
            self.rotZ = 0
        else :
            self.rotZ = val
        
    def initializeGL(self):
        f = self.context().functions()
        self.initGeometry()
        self.rotX = 0.0
        self.rotY = 0.0
        self.rotZ = 0.0
        f.glEnable(GL_DEPTH_TEST)
        f.glClearColor(0.0,0.0,0.0,1.0)
    def paintGL(self):
        f = self.context().functions()
        self.makeCurrent()
        
        glClearColor(0.0,0.0,0.0,1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()
        glTranslate(0.0,0.0,-50.0)
        glScale(20.0,20.0,20.0)
        glRotate(self.rotX, 1.0, 0.0, 0.0)
        glRotate(self.rotY, 0.0, 1.0, 0.0)
        glRotate(self.rotZ, 0.0, 0.0, 1.0)
        glTranslate(-0.5,-0.5,-0.5)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3,GL_FLOAT,0,self.vertVBO)
        glColorPointer(3,GL_FLOAT,0,self.colorVBO)
        glDrawElements(GL_QUADS,len(self.cubeIdxArray), GL_UNSIGNED_INT, self.cubeIdxArray)
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glPopMatrix()
        
    def resizeGL(self, width, height):
        f = self.context().functions()
        self.makeCurrent()
        glViewport(0,0,width,height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width/float(height)
        GLU.gluPerspective(45.0,aspect, 1.0, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        
    def advance(self):
        #self.rotX += 1.0
        #self.rotY += 2.0
        #self.rotZ += 3.0
        self.update()


class Custom_MainWidget(MWB, QWidget):
    def __init__(self, params):
        MWB.__init__(self, params)
        QWidget.__init__(self)

        self.setLayout(QVBoxLayout())

        self.img_view = Open_MainWidget(params)
        self.glWidget = CustomGL_CodeWidget()
        self.glWidget.resize(400,400)
        self.layout().addWidget(self.img_view)
        self.layout().addWidget(self.glWidget)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.advance)
        self.timer.start(20)
        self.valx = 0
        self.valy = 0
        self.valz = 0

    def setRotXYZ(self):
        if self.node.animating == True : 
            self.valx += self.node.rX
            self.valy += self.node.rY
            self.valz += self.node.rZ
        else :
            self.valx = self.node.rX
            self.valy = self.node.rY
            self.valz = self.node.rZ
        self.glWidget.setRotX(self.valx)
        self.glWidget.setRotY(self.valy)
        self.glWidget.setRotZ(self.valz)
        
    def advance(self):
        self.setRotXYZ()
        self.glWidget.advance()
        self.update_node()


    def get_state(self) -> dict:
        return {
            'text': self.editor.toPlainText(),
        }

    def set_state(self, data: dict):
        self.editor.setPlainText(data['text'])




class FileInput(IWB, QWidget):
    path_chosen = Signal(str)

    def __init__(self, params):
        IWB.__init__(self, params)
        QWidget.__init__(self)

        self.path = ''

        # setup UI
        l = QVBoxLayout()
        button = QPushButton('choose')
        button.clicked.connect(self.choose_button_clicked)
        l.addWidget(button)
        self.path_label = QLabel('path')
        l.addWidget(self.path_label)
        self.setLayout(l)
    
    def choose_button_clicked(self):
        abs_f_path, filter = QFileDialog.getOpenFileName(self, "Select File")
        self.path = abs_f_path

        self.path_label.setText(os.path.relpath(abs_f_path))
        self.adjustSize()  # important! otherwise the widget won't shrink

        self.path_chosen.emit(self.path)

        self.node.update_shape()

    def get_state(self):
        return {'path': self.path}
    
    def set_state(self, data):
        self.path = data['path']
        self.path_label.setText(self.path)
        self.node.update_shape()

class PathInput(IWB, QWidget):
    path_chosen = Signal(str)

    def __init__(self, params):
        IWB.__init__(self, params)
        QWidget.__init__(self)

        self.path = ''

        # setup UI
        l = QVBoxLayout()
        button = QPushButton('choose')
        button.clicked.connect(self.choose_button_clicked)
        l.addWidget(button)
        self.path_label = QLabel('path')
        l.addWidget(self.path_label)
        self.setLayout(l)
    
    def choose_button_clicked(self):
        abs_f_path = folder = str(QFileDialog.getExistingDirectory(None, "Select Directory"))
        self.path = os.path.relpath(abs_f_path)

        self.path_label.setText(self.path)
        self.adjustSize()  # important! otherwise the widget won't shrink

        self.path_chosen.emit(self.path)

        self.node.update_shape()

    def get_state(self):
        return {'path': self.path}
    
    def set_state(self, data):
        self.path = data['path']
        self.path_label.setText(self.path)
        self.node.update_shape()

class CodeNode_MainWidget(MWB, QTextEdit):
    def __init__(self, params):
        MWB.__init__(self, params)
        QTextEdit.__init__(self)

        self.setFont(QFont('Consolas', 9))
        #self.textChanged.connect(self.text_changed)
        self.setFixedHeight(150)
        self.setFixedWidth(300)
        self.setReadOnly(True)
        
    def update_text(self, text):
        if text is None:
            self.setText('')
            return
        self.setText(text)
        print("widget: setting:"+text)
        self.update_node()
        

    def get_state(self) -> dict:
        return {
            'text': self.node.code,
        }

    def set_state(self, data: dict):
        print('widget:set_state')
        self.setPlainText(data['text'])



#---------------------------------- opencv stuff

import cv2
import os


class OpenCVNode_MainWidget(MWB, QLabel):
    def __init__(self, params):
        MWB.__init__(self, params)
        QLabel.__init__(self)

        self.resize(200, 200)

    def show_image(self, img):
        self.resize(200, 200)
        self.is16bit = False
        d16 = np.dtype('uint16')
        print('opencvnode')
        print(img.dtype)
        try:
    
            if img.dtype == d16 :
                self.is16bit = True
                print('16 bit')
                im2 = (img/256).astype('uint8')
                im3 = im2.copy()
                cv2.normalize(im2,im3, 0, 255, cv2.NORM_MINMAX)
                rgb_image = cv2.cvtColor(im3, cv2.COLOR_GRAY2RGB)
            else :
                self.is16bit = False
                print('8 bit')
                if len(img.shape) < 3:
                    print('grayscale')
                    rgb_image = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                else :
                    rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except cv2.error:
            return

        h, w, ch = rgb_image.shape
        print("w:"+str(w))
        print("h:"+str(h))
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        img_w = qt_image.width()
        img_h = qt_image.height()
        proportion = img_w / img_h
        self.resize(self.width() * proportion, self.height())
        qt_image = qt_image.scaled(self.width(), self.height())
        self.setPixmap(QPixmap(qt_image))
        self.node.update_shape()






#------------------------------- export




export_widgets(
    Custom_MainWidget,
    ButtonNode_MainWidget,
    PathInput, FileInput, 
    CodeNode_MainWidget,
    OpenCVNode_MainWidget
)
