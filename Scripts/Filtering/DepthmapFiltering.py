import cv2 as cv
import numpy as np

def ShowImage(Mat, showColor):
    # Normalize the 16 bit depth image to 8 bit
    Mat = cv.normalize(Mat, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)

    if(showColor):
        Mat = cv.applyColorMap(Mat, cv.COLORMAP_MAGMA)

    cv.imshow("Depth map window", Mat)
    k = cv.waitKey(0)



path = "C:/Users/Christopher/Desktop/Depthmap_Filter_Test/"
depthMat = cv.imread(path + "Depth_62.tiff", cv.IMREAD_ANYDEPTH)

if(depthMat is None):
    print("Image not found")
    quit()

ShowImage(depthMat, True)

# Double the size of the image
depthMat = cv.resize(depthMat, None, (0,0), fx=2, fy=2, interpolation=cv.INTER_NEAREST)



blur = cv.blur(depthMat, (5,5))

#ShowImage(blur, True)

diff = cv.absdiff(blur, depthMat, None)

#ShowImage(diff, True)

ret, tres = cv.threshold(diff, 23, 255, 4)

#ShowImage(tres, False)

tres = cv.normalize(tres, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)

blur2 = cv.blur(tres, (3,3))

ret2, tres2 = cv.threshold(blur2, 30, 255, 0)

#ShowImage(tres2, False)



mask = cv.bitwise_not(tres2)
masked = cv.bitwise_and(depthMat, depthMat, mask= mask)


masked = cv.resize(masked, None, (0,0), fx = 0.5, fy = 0.5, interpolation=cv.INTER_NEAREST)

ShowImage(masked, True)



