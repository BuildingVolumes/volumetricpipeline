import PySimpleGUI as sg
import os
import io
import json
import cv2
import glob
import numpy as np
from PIL import Image


# rgbmode = "";
# depthmode = "";

# SyncMode = "";
# SyncIndex = "";

# OutputFolder = "";
# FilePrefix = "";
# Filename = FilePrefix + SyncIndex + ".mkv"
# CMD = "";
# fps = "";
# config = {"rgbmode":rgbmode, "depthmode":depthmode, "SyncMode":SyncMode, "SyncIndex":SyncIndex, "OutputFolder":OutputFolder, "FilePrefix":FilePrefix, "CMD":CMD, "fps":fps}


# def write_config_file():
#     global config
#     #config = {"rgbmode":rgbmode, "depthmode":depthmode, "SyncMode":SyncMode, "SyncIndex":SyncIndex, "OutputFolder":OutputFolder, "FilePrefix":FilePrefix, "CMD":CMD, "fps":fps}
#     with open('config1.json','w') as f:
#         json.dump(config,f);

# def read_config_file():
#     global config
#     with open('config1.json','r') as f:
#         config = json.load(f);
    

# def print_all():
#     global config
#     print('rgbmode:'+config['rgbmode'])
#     print('depthmode:'+config['depthmode'])
#     print('SyncMode:'+config['SyncMode'])
#     print('SyncIndex:'+config['SyncIndex'])
#     print('OutputFolder:'+config['OutputFolder'])
#     print('FilePrefix:'+config['FilePrefix'])
#     print('CMD:'+config['CMD'])
#     print('fps:'+config['fps'])


# def set_all(valuesT):
#     global config;
#     print(config)
    
#     config['rgbmode'] = valuesT[1];
#     config['depthmode'] = valuesT[2];
#     config['SyncMode'] = valuesT[3];
#     config['SyncIndex'] = valuesT[4];
#     config['OutputFolder'] = valuesT[5];
#     config['FilePrefix'] = valuesT[6];
#     config['CMD'] = valuesT[7];
#     config['fps'] = valuesT[8];
#     print_all()



# always read the config first
#read_config_file();
#print_all()

############################
# This is a GUI for OpenCV's Camera Calibration
# - purpose: to calibrate a SINGLE camera intrinsics
############################
# functionality
# + select folder directory to find images
# + list all images in a list, select them to show 
# - add/remove images from list
# + Find chessboard corners/calibration target in all images in list
# - setup calibration target info
# - draw calibration target toggle
# - Button to run calibration
# - save calibration in reasonable format
# - save rectified images button

# INPUT_FOLDER = r'./'
# files = [os.path.join(INPUT_FOLDER,f) for f in os.listdir(INPUT_FOLDER) if f.endswith('png')]

# layout = [ [sg.Text('Click Image For Next')],
#            [sg.Image(filename=files[0], key='-IMAGE-', enable_events=True)]]


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
NumX = 9
NumY = 6

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((NumY*NumX,3), np.float32)
objp[:,:2] = np.mgrid[0:NumX,0:NumY].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

left_col = [[sg.Text('Folder'), sg.In(size=(25,1), enable_events=True ,key='-FOLDER-'), sg.FolderBrowse(initial_folder='.')],
            [sg.Listbox(values=[], enable_events=True, size=(40,20),key='-FILE LIST-')]]

# For now will only show the name of the file that was chosen
images_col = [[sg.Text('You choose from the list:')],
              [sg.Text(size=(40,1), key='-TOUT-')],
              [sg.Image(key='-IMAGE-', size=(400,400))]]

# ----- Full layout -----
layout = [[sg.Frame(layout=[
                    [sg.Column(left_col), sg.VSeperator(), sg.Column(images_col)],
                    [sg.Button(button_text='Calibrate (all images)', key='-B-CALIBRATE-')]              
                    ], title='Settings',title_color='white', relief=sg.RELIEF_SUNKEN)],
            [sg.Frame(layout=[
                [sg.Text('Num Squares X', size=(15, 1)), sg.InputText(key='NX',size=(3,1),default_text=str(NumX))],
                [sg.Text('Num Squares Y', size=(15, 1)), sg.InputText(key='NY', size=(3,1), default_text=str(NumY))],
                [sg.Button(button_text='Find Calibration Target (selected image)', key='-B-FINDTARGET-')],
            ], title='Calibration Target Info',title_color='white', relief=sg.RELIEF_SUNKEN)],

            [sg.Multiline(key='-TOUTPUT-'+sg.WRITE_ONLY_KEY,size=(200,25))]]
            
# --------------------------------- Create Window ---------------------------------
window = sg.Window('Image Viewer', layout)

debugOutput=""
CalibIntrinsicsMatrix = []
CalibIntrinsicsDist =[]
Calibrated = False

def mprint(*args, **kwargs):
    window['-TOUTPUT-' + sg.WRITE_ONLY_KEY].print(*args, **kwargs)

print = mprint

# ----- Run the Event Loop -----
# --------------------------------- Event Loop ---------------------------------
while True:
    event, values = window.read()

    if event in (None, 'Exit'):
        break
    if event == '-FOLDER-':                     # Folder name was filled in, make a list of files in the folder
        folder = values['-FOLDER-']
        try:
            file_list = os.listdir(folder)         # get list of files in folder
        except:
            file_list = []

        fnames = [f for f in file_list if os.path.isfile(
            os.path.join(folder, f)) and f.lower().endswith((".png", ".jpg", "jpeg", ".tiff", ".bmp"))]
        window['-FILE LIST-'].update(fnames)
    elif event == '-FILE LIST-':    # A file was chosen from the listbox
        #try:
            filename = os.path.join(values['-FOLDER-'], values['-FILE LIST-'][0])
            img = cv2.imread(filename)
            #image = Image.open(filename)
            #image.thumbnail((400,400))
            #bio = io.BytesIO();
            imgbytes = cv2.imencode('.png',img)[1].tobytes()
            #image.save(bio,format="PNG")
            window['-TOUT-'].update(filename)
            #window['-IMAGE-'].update(data=bio.getvalue())
            window['-IMAGE-'].update(data=imgbytes)
            if Calibrated == True:
                print("Undistorting image")
                h,w = img.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(CalibIntrinsicsMatrix, CalibIntrinsicsDist, (w,h),1,(w,h))
                dst = cv2.undistort(img,CalibIntrinsicsMatrix,CalibIntrinsicsDist, None, newcameramtx)
                x,y,w,h = roi
                dst = dst[y:y+h,x:x+w]
                imgbytes = cv2.imencode('.png',dst)[1].tobytes()
                window['-IMAGE-'].update(data=imgbytes)
    elif event == '-B-FINDTARGET-':
            NumX = int(values['NX'])
            NumY = int(values['NY'])
            objp = np.zeros((NumY*NumX,3), np.float32)
            objp[:,:2] = np.mgrid[0:NumX,0:NumY].T.reshape(-1,2)
            objpoints = [] 
            imgpoints = [] 
            filename = os.path.join(values['-FOLDER-'], values['-FILE LIST-'][0])
            img = cv2.imread(filename)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            imgbytes = cv2.imencode('.png',gray)[1].tobytes()
            window['-IMAGE-'].update(data=imgbytes) 
            ret,corners = cv2.findChessboardCorners(gray,(NumX,NumY),None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img,(NumX,NumY),corners2, ret)
                imgbytes = cv2.imencode('.png',img)[1].tobytes()
                window['-IMAGE-'].update(data=imgbytes)  
    elif event == '-B-CALIBRATE-':
        NumX = int(values['NX'])
        NumY = int(values['NY'])
        objp = np.zeros((NumY*NumX,3), np.float32)
        objp[:,:2] = np.mgrid[0:NumX,0:NumY].T.reshape(-1,2)
        objpoints = [] 
        imgpoints = [] 
        folder = values['-FOLDER-']
        file_list = os.listdir(folder) 
        for f in file_list:
            filename = os.path.join(values['-FOLDER-'],f)
            print(filename)
            img = cv2.imread(filename)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret,corners = cv2.findChessboardCorners(gray,(NumX,NumY),None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img,(NumX,NumY),corners2, ret)
                imgbytes = cv2.imencode('.png',img)[1].tobytes()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        CalibIntrinsicsMatrix=mtx
        CalibIntrinsicsDist = dist
        Calibrated = True
        print("Calibrated!")

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(objpoints)) )
        print(mtx)
        print(dist)

    window['-TOUTPUT-'+sg.WRITE_ONLY_KEY].expand(True,True)

# --------------------------------- Close & Exit ---------------------------------

window.close()
