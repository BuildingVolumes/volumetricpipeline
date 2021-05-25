import PySimpleGUI as sg
import os
import io
import json
import cv2
import glob
import numpy as np
from PIL import Image

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



########################################
## CameraCalibrationIntrinsics
## -- just stores the intrinsic matrix and distortions
########################################
class CameraCalibrationIntrinsics:
    """stores intrinsics and distortions only for one camera"""
    def __init__(self):
        self.mtx = []
        self.dist = []

########################################
## CalibrationImage
## -- data storage for rgb, grayscale, corners, etc
########################################
class CalibrationImage:
    def __init__(self, filename):
        self.image = []
        self.filename = filename
        self.gray = []
        self.undistorted = []
        self.corners = []
        self.image_withCorners = []
        self.objp = []        
        self.image = cv2.imread(filename)
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image_withCorners = self.image.copy()
        self.cornersFound = False

    def findChessBoard(self, numx, numy):
        self.objp = np.zeros((NumY*NumX,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:NumX,0:NumY].T.reshape(-1,2)
        ret,corners = cv2.findChessboardCorners(self.gray,(numx,numy),None)
        if ret == True:
            corners2 = cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),criteria)
            self.corners = corners2
            cv2.drawChessboardCorners(self.image_withCorners,(numx,numy),self.corners, ret)
            self.cornersFound = True
        else:
            print("CHESSBOARD NOT FOUND: "+self.filename)

########################################
## CameraCalibration
## -- the calibrator
########################################    
class CameraCalibration:
    """performs intrinsic calibration using opencv"""
    def __init__(self):
        self.intrinsics = CameraCalibrationIntrinsics()
        self.targetInfo = []
        self.Calibrated = False
        self.objpoints = [] # 3d point in real world space
        self.imagepoints = [] # 2d points in image plane.
        self.imageFilenames = []
        self.images = []
        self.dict_corners_objp = {}
        self.dict_corners_c2 = {}
        self.dictImages = {}
        self.sz = []

    def save(self, fileName):
        """save calibration info """
    def load(self,fileName):
        """load calibration info """
    def calibrate(self):
        """performs the calibration """
    def undistortImage(self,img):
        """applies the calibration info to the passed in img"""

    def addImage(self,filename):
        """adds an image to the calibration list"""
        if (filename in self.imageFilenames) == False:
            # not already added to the list
            # so we should load and add to list
            print("Loading:"+filename)
            self.imageFilenames.append(filename)
            theImage = CalibrationImage(filename);
            self.dictImages[filename] = theImage;
            self.sz = theImage.gray.shape[::-1]; 
        else:
            print("File already loaded: "+filename)
            theImage = self.dictImages[filename]
            # already in list
            # get index 
        if theImage.cornersFound == True:
            return theImage.image_withCorners
        else:
            return theImage.image

    def findChessBoard(self, filename):
        print("findChessBoard("+filename+")")
        """ find the chessboard in the passed image """
        if (filename in self.imageFilenames) == True:
            theImage = self.dictImages[filename]
            theImage.findChessBoard(self.targetInfo.NumX,self.targetInfo.NumY)
            return theImage.image_withCorners
        print("ERROR:findChessBoard("+filename+")")

    def calibrate(self):
        try:
            if self.Calibrated == True:
                print("calibrate(): already calibrated!")
                print("CameraIntrinsics:", self.intrinsics.mtx)
                print("Distortion:",self.intrinsics.dist)
                return
            self.objpoints = [] # 3d point in real world space
            self.imgpoints = [] # 2d points in image plane.
            NumX = self.targetInfo.NumX
            NumY = self.targetInfo.NumY
            GRIDSIZE = self.targetInfo.SquareSize
            objp = np.zeros((NumY*NumX,3), np.float32)
            objp[:,:2] = np.mgrid[0:NumX,0:NumY].T.reshape(-1,2)*GRIDSIZE
            #sz = self.dictImages[1].gray.shape[::-1]
            print("calibrate()")
            for fname in self.imageFilenames:
                print(fname)
                theImage = self.dictImages[fname]
                self.objpoints.append(objp)
                self.imagepoints.append(theImage.corners)
       
            print("---cv2.calibrateCamera()")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imagepoints, self.sz, None, None)
            self.intrinsics.mtx=mtx
            self.intrinsics.dist = dist
            self.Calibrated = True
            mean_error = 0
            print("---checking reprojection error")
            length = len(self.objpoints)
            for i in range(length):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], self.intrinsics.mtx, self.intrinsics.dist)
                error = cv2.norm(self.imagepoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error       
            print("CameraIntrinsics:", self.intrinsics.mtx)
            print("Distortion:",self.intrinsics.dist)
            print( "Reprojection Error: {}".format(mean_error/length))
        except:
            print("calibrate()::err")
    def save(self,filename):
        print("save:"+filename)
        m = self.intrinsics.mtx.tolist()
        d = self.intrinsics.dist.tolist()
        data = {"camera_matrix":m, "distortions":d}
        with open(filename,'w') as f:
            json.dump(data,f);
    def load(self,filename):
        print("LOAD:"+filename)
        with open(filename,'r') as f:
            data = json.load(f);
        self.intrinsics.mtx = np.asarray(data["camera_matrix"])
        self.intrinsics.dist = np.asarray(data["distortions"])
        print("CameraIntrinsics:", self.intrinsics.mtx)
        print("Distortion:",self.intrinsics.dist)

    def setCalibrationTarget(self, cTargetInfo):
        self.targetInfo = cTargetInfo
    def setCalibrationTarget(self, numx,numy, ss):
        self.targetInfo = CalibrationTargetInfo(numx,numy, ss)


############################
# This is a GUI for OpenCV's Camera Calibration
# - purpose: to calibrate a SINGLE camera intrinsics
############################
# functionality
# + select folder directory to find images
# + list all images in a list, select them to show 
# - add/remove images from list
# + Find chessboard corners/calibration target in all images in list
# + setup calibration target info
# - draw calibration target toggle
# + Button to run calibration
# - save calibration in reasonable format
# - save rectified images button

def getThumbnail(img, hh):
    width = img.shape[1]
    height = img.shape[0]
    newHeight = hh;
    scale = newHeight/height
    newWidth = int(width * scale)
    imgThumbnail = cv2.resize(img, (newWidth,newHeight), interpolation=cv2.INTER_AREA)
    return imgThumbnail


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
NumX = 9
NumY = 6
SSize = 1

left_col = [[sg.Text('Folder'), sg.In(size=(25,1), enable_events=True ,key='-FOLDER-'), sg.FolderBrowse(initial_folder='.')],
            [sg.Listbox(values=[], enable_events=True, size=(40,20),key='-FILE LIST-')]]

# For now will only show the name of the file that was chosen
images_col = [[sg.Text('You choose from the list:')],
              [sg.Text(size=(40,1), key='-TOUT-')],
              [sg.Image(key='-IMAGE-', size=(400,400))]]

# ----- Full layout -----
layout = [[sg.Frame(layout=[
                    [sg.Column(left_col), sg.VSeperator(), sg.Column(images_col)],
                    [sg.Button(button_text='Calibrate (all images)', key='-B-CALIBRATE-')],
                    [sg.Text('Calibration Filename', size=(20, 1)), sg.InputText(default_text='intrinsics.json', key="-T-CALIBFNAME-")],
                    [sg.Button(button_text='Save Calibration', key="-B-SAVECALIB-")],
                    [sg.Button(button_text='Load Calibration', key="-B-LOADCALIB-")],
                    ], title='Settings',title_color='white', relief=sg.RELIEF_SUNKEN)],
            [sg.Frame(layout=[
                [sg.Text('Num Squares X  ', size=(15, 1)), sg.InputText(key='NX',size=(3,1),default_text=str(NumX))],
                [sg.Text('Num Squares Y  ', size=(15, 1)), sg.InputText(key='NY', size=(3,1), default_text=str(NumY))],
                [sg.Text('Square Size(mm)', size=(15, 1)), sg.InputText(key='SS', size=(3,1), default_text=str(SSize))],\
                [sg.Button(button_text='Find Calibration Target (selected image)', key='-B-FINDTARGET-')],
                [sg.Button(button_text='Find ALL Targets (all images)', key='-B-FINDTARGET-ALL-')],
            ], title='Calibration Target Info',title_color='white', relief=sg.RELIEF_SUNKEN)],

            [sg.Multiline(key='-TOUTPUT-'+sg.WRITE_ONLY_KEY,size=(200,25))]]
            
# --------------------------------- Create Window ---------------------------------
window = sg.Window('Image Viewer', layout)

def mprint(*args, **kwargs):
    window['-TOUTPUT-' + sg.WRITE_ONLY_KEY].print(*args, **kwargs)

print = mprint

calibrator = CameraCalibration()
calibrator.setCalibrationTarget(NumX,NumY,1)

# ----- Run the Event Loop -----
# --------------------------------- Event Loop ---------------------------------
while True:
    try:
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
                filename = os.path.join(values['-FOLDER-'], values['-FILE LIST-'][0])
                print(filename)
                img = calibrator.addImage(filename)
                imgThumbnail = getThumbnail(img,400)
                imgbytes = cv2.imencode('.png',imgThumbnail)[1].tobytes()
                window['-TOUT-'].update(filename)
                window['-IMAGE-'].update(data=imgbytes)
        elif event == '-B-FINDTARGET-':
                NumX = int(values['NX'])
                NumY = int(values['NY'])
                SSize = int(values['SS'])
                calibrator.setCalibrationTarget(NumX,NumY, SSize)

                filename = os.path.join(values['-FOLDER-'], values['-FILE LIST-'][0])
                print(filename)
                img = calibrator.findChessBoard(filename)
                imgThumbnail = getThumbnail(img,400)
                imgbytes = cv2.imencode('.png',imgThumbnail)[1].tobytes()
                window['-IMAGE-'].update(data=imgbytes)  
        elif event == '-B-FINDTARGET-ALL-':
            NumX = int(values['NX'])
            NumY = int(values['NY'])
            SSize = int(values['SS'])
            calibrator.setCalibrationTarget(NumX,NumY, SSize)
            folder = values['-FOLDER-']
            file_list = os.listdir(folder) 
            fnames = [f for f in file_list if os.path.isfile(
                os.path.join(folder, f)) and f.lower().endswith((".png", ".jpg", "jpeg", ".tiff", ".bmp"))]
            for f in fnames:
                 filename = os.path.join(values['-FOLDER-'],f)
                 print(filename)
                 calibrator.addImage(filename)
                 calibrator.findChessBoard(filename)
        elif event == '-B-CALIBRATE-':
            NumX = int(values['NX'])
            NumY = int(values['NY'])
            SSize = int(values['SS'])
            calibrator.setCalibrationTarget(NumX,NumY, SSize)
            folder = values['-FOLDER-']
            file_list = os.listdir(folder) 
            fnames = [f for f in file_list if os.path.isfile(
                os.path.join(folder, f)) and f.lower().endswith((".png", ".jpg", "jpeg", ".tiff", ".bmp"))]
            for f in fnames:
                 filename = os.path.join(values['-FOLDER-'],f)
                 print(filename)
                 calibrator.addImage(filename)
                 calibrator.findChessBoard(filename)
            calibrator.calibrate()
        elif event == '-B-SAVECALIB-':
            filename = os.path.join(values['-FOLDER-'], values['-T-CALIBFNAME-'])
            calibrator.save(filename)
        elif event == '-B-LOADCALIB-':
            filename = os.path.join(values['-FOLDER-'], values['-T-CALIBFNAME-'])
            calibrator.load(filename)

        window['-TOUTPUT-'+sg.WRITE_ONLY_KEY].expand(True,True)
    except Exception as e:
        print(e)
# --------------------------------- Close & Exit ---------------------------------

window.close()
