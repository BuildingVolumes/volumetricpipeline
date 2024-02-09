import cv2 as cv
import numpy as np
import time
import os
import FileManagement as fm

def ShowImage(Mat, showColor):
    # Normalize the 16 bit depth image to 8 bit
    Mat = cv.normalize(Mat, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)

    if(showColor):
        Mat = cv.applyColorMap(Mat, cv.COLORMAP_MAGMA)

    cv.imshow("Depth map window", Mat)
    k = cv.waitKey(0)
    if(k == ord('q')):
        quit()

# Get all .tiff images from folder into a path list and sort in numerical order
path = "C:/Users/Christopher/Desktop/Depthmap_Filter_Test/TempFilter/client_0/"

files = fm.GetFiles(path, ".tiff")
files = fm.SortFileList(files)

outputDir = fm.CreateOutputDir(path, "TempFilter")

averageImg = cv.imread(path + "/" + files[0], cv.IMREAD_ANYDEPTH).astype(np.float32)
for f in files:
    img = cv.imread(path + "/" + f, cv.IMREAD_ANYDEPTH).astype(np.float32)
    cv.accumulateWeighted(img, averageImg, 0.9)
    digit = fm.GetDigitFromFilename(f)

    #Convert back to 16 bit and save
    depthImage16 = (averageImg).astype(np.uint16)
    cv.imwrite(outputDir + "/synced_Depth_" + digit + ".tiff", depthImage16)