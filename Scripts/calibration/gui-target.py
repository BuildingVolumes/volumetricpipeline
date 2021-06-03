import PySimpleGUI as sg
import os
import io
import json
import cv2
import glob
import numpy as np
import PIL.Image
from cairosvg import svg2png

########################################
## CalibrationTargetInfo
## -- just stores nx/ny corners at the moment 
## -- could be used for different kinds of targets
########################################
class CalibrationTargetInfo:
    """Settings for the calibration target"""
    def __init__(self):
        self.NumX = 9
        self.NumY = 6
        self.SquareSize = 1
    def __init__(self,_x,_y,_s=1):
        self.NumX = _x
        self.NumY = _y
        self.SquareSize = _s




def getThumbnail(img, hh):
    width = img.shape[1]
    height = img.shape[0]
    newHeight = hh;
    scale = newHeight/height
    newWidth = int(width * scale)
    imgThumbnail = cv2.resize(img, (newWidth,newHeight), interpolation=cv2.INTER_AREA)
    return imgThumbnail


def convert_to_bytes(file_or_bytes, resize=None):
    '''
    Will convert into bytes and optionally resize an image that is a file or a base64 bytes object.
    Turns into  PNG format in the process so that can be displayed by tkinter
    :param file_or_bytes: either a string filename or a bytes base64 image object
    :type file_or_bytes:  (Union[str, bytes])
    :param resize:  optional new size
    :type resize: (Tuple[int, int] or None)
    :return: (bytes) a byte-string object
    :rtype: (bytes)
    '''
    if isinstance(file_or_bytes, str):
        img = PIL.Image.open(file_or_bytes)
    else:
        try:
            img = PIL.Image.open(io.BytesIO(base64.b64decode(file_or_bytes)))
        except Exception as e:
            dataBytesIO = io.BytesIO(file_or_bytes)
            img = PIL.Image.open(dataBytesIO)

    cur_width, cur_height = img.size
    if resize:
        new_width, new_height = resize
        scale = min(new_height/cur_height, new_width/cur_width)
        img = img.resize((int(cur_width*scale), int(cur_height*scale)), PIL.Image.ANTIALIAS)
    bio = io.BytesIO()
    img.save(bio, format="PNG")
    del img
    return bio.getvalue()
    
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
NumX = 9
NumY = 6
SSize = 25
W = 20
H = 20
RADRATE = 5

left_col = [[sg.Text('Folder'), sg.In(size=(25,1), enable_events=True ,key='-FOLDER-'), sg.FolderBrowse(initial_folder='.')],
            [sg.Listbox(values=[], enable_events=True, size=(40,20),key='-FILE LIST-')]]

# For now will only show the name of the file that was chosen
images_col = [[sg.Text('Calibration Target Preview')],
              [sg.Text(size=(40,1), key='-TOUT-')],
              [sg.Image(key='-IMAGE-', size=(800,400),data=convert_to_bytes("temp.png", resize=(800,400)))]]
Col_W=20
target_info = [[sg.Frame(layout=[
                [sg.Text('Output Prefix ', size=(Col_W, 1)), sg.InputText(key='FILENAME',size=(Col_W,1),default_text='output')],
                [sg.Text('Num Corners X  ', size=(Col_W, 1)), sg.InputText(key='NX',size=(3,1),default_text=str(NumX))],
                [sg.Text('Num Corners Y  ', size=(Col_W, 1)), sg.InputText(key='NY', size=(3,1), default_text=str(NumY))],
                [sg.Text('Square Size', size=(Col_W, 1)), sg.InputText(key='SS', size=(3,1), default_text=str(SSize))],
                [sg.Text('Radius Rate', size=(Col_W, 1)), sg.InputText(key='-RRATE', size=(3,1), default_text=str(RADRATE))],
                [sg.Combo(['checkerboard','circles','acircles'] , default_value='checkerboard', size=(Col_W,1), key='-TYPE')],
                [sg.Combo(['mm','inches','px','m'] , default_value='mm', size=(Col_W,1), key='-UNIT')],
                [sg.Button(button_text='Create Target', key='-B-TARGET')]
            ], title='Calibration Target Info',title_color='white', relief=sg.RELIEF_SUNKEN)]]

# ----- Full layout -----
layout = [[sg.Column(target_info, vertical_alignment='top'), sg.Column(images_col)],
            [sg.Text('Command Output:')],
            [sg.Multiline(key='-TOUTPUT-'+sg.WRITE_ONLY_KEY,size=(200,25))]]
# layout = [[sg.Frame(layout=[
#                    [ sg.Column(images_col)],
#                    ],title='Settings',title_color='white', relief=sg.RELIEF_SUNKEN)],
#            [sg.Multiline(key='-TOUTPUT-'+sg.WRITE_ONLY_KEY,size=(200,25))]]
#
# --------------------------------- Create Window ---------------------------------
window = sg.Window('Image Viewer', layout)

def mprint(*args, **kwargs):
    window['-TOUTPUT-' + sg.WRITE_ONLY_KEY].print(*args, **kwargs)



print = mprint


#svg2png(open("temp.svg", 'rb').read(), write_to=open("temp.png", 'wb'))



# ----- Run the Event Loop -----
# --------------------------------- Event Loop ---------------------------------
while True:
    try:
        event, values = window.read()
        window['-IMAGE-'].update(data=convert_to_bytes("temp.png",resize=(800,400)))

        if event in (None, 'Exit'):
            break
        else:
            CMD = "opencv/pattern_tools/gen_pattern.py"
            
            theWidth = (int(values['NX'])+1)*int(values['SS'])
            theHeight = (int(values['NY'])+1)*int(values['SS'])
            FNAME =values['FILENAME']+"_"+values['NX']+"x"+values['NY']+"_"+values['SS']+"mm.svg"
            OUTPUT=" -o " + FNAME
            Width = " -w "+ str(theWidth)
            Height = " -h "+ str(theHeight)
            UNIT = " -u "+values['-UNIT']
            NumX = " --rows " + str(int(values['NY'])+1)
            NumY = " --columns " + str(int(values['NX'])+1)
            SS = " --square_size " + values['SS']
            TYPE = " --type " +values['-TYPE']
            RADRATE = " --radius_rate "+values['-RRATE']
            ARGS = " " + OUTPUT + NumX + NumY + SS + TYPE + Width + Height +UNIT + RADRATE
            FNAMEPNG=FNAME+".png"
            print("running: "+CMD + ARGS)
            os.system(CMD + ARGS)
            svg2png(open(FNAME, 'rb').read(), write_to=open(FNAMEPNG, 'wb'))
            window['-IMAGE-'].update(data=convert_to_bytes(FNAMEPNG,resize=(800,400)))

        window['-TOUTPUT-'+sg.WRITE_ONLY_KEY].expand(True,True)
    except Exception as e:
        print(e)
# --------------------------------- Close & Exit ---------------------------------

window.close()
