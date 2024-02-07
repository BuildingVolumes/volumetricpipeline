import cv2 as cv
import numpy as np
import time
import os


def ShowImage(Mat, showColor):
    # Normalize the values from o to 64000 to 0 to 255
    max = 2000 # Show max depth values 5 meter far away
    Mat = cv.convertScaleAbs(Mat, None, 255 / max)

    if(showColor):
        Mat = cv.applyColorMap(Mat, cv.COLORMAP_JET)

    cv.imshow("Depth map window", Mat)


# Get all .tiff images from folder into a path list and sort in numerical order
path = "C:/Users/Christopher/Desktop/Depthmap_Filter_Test/client_0"
tempfilteredFolder = "tempFiltered"
tempFilteredPath = path + "/" + tempfilteredFolder

if(os.path.exists(tempFilteredPath)):

    files = os.listdir(tempFilteredPath)
    # Delete all files in the folder
    for f in files:
        os.remove(tempFilteredPath + "/" + f)

    os.removedirs(tempFilteredPath)

os.makedirs(tempFilteredPath)

files = os.listdir(path)
files = [f for f in files if f.endswith('.tiff')]
files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

averageImg = cv.imread(path + "/" + files[0], cv.IMREAD_ANYDEPTH).astype(np.float32)

for f in files[1:]:
    img = cv.imread(path + "/" + f, cv.IMREAD_ANYDEPTH).astype(np.float32)
    cv.accumulateWeighted(img, averageImg, 0.9)
    #ShowImage(averageImg, True)
    digit = f.split(".")[0]
    digit = f.split("_")[1]
    cv.imwrite(tempFilteredPath + "/Depth_" + digit + ".tiff", averageImg)
    cv.waitKey(3)