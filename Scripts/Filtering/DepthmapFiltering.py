import cv2 as cv
import numpy as np
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


path = "C:/Users/Christopher/Desktop/Depthmap_Filter_Test/FlyingPixel/client_0/"

files = fm.GetFiles(path, ".tiff")
files = fm.SortFileList(files)

outputDir = fm.CreateOutputDir(path, "FlyingPixelRemoval")

for f in files:

    depthMat = cv.imread(path + f, cv.IMREAD_ANYDEPTH)

    if(depthMat is None):
        print("Image not found: " + str(f))
        continue

    #ShowImage(depthMat, True)

    # Double the size of the image, so that blurring is more precise
    depthMat = cv.resize(depthMat, None, (0,0), fx=2, fy=2, interpolation=cv.INTER_NEAREST)

    # Blur to get the average value of the neighbor pixels
    blur = cv.blur(depthMat, (5,5))
    #ShowImage(blur, True)

    # Calculate the difference in distance between the pixel and its neighbors
    diff = cv.absdiff(blur, depthMat, None)
    #ShowImage(diff, True)

    # Get all the pixels that are more than 23mm away from the average value of the neighbor pixels
    # These are our flying pixels
    ret, tres = cv.threshold(diff, 10, 255, 4)
    #ShowImage(tres, False)

    # Create a boolean mask from the flying pixel map, blur it to make it smoother
    tres = cv.normalize(tres, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)
    blur2 = cv.blur(tres, (5,5))
    ret2, tres2 = cv.threshold(blur2, 20, 255, 0)
    #ShowImage(tres2, False)

    dilationShape = 1
    dilatation_size = 2
    element = cv.getStructuringElement(1, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    dilatated = cv.dilate(tres2, element)

    # Invert the mask, so that the flying pixels are black and the rest is white
    mask = cv.bitwise_not(dilatated)
    ShowImage(mask, False)

    # Remove the flying pixels from the original depth image using the mask
    masked = cv.bitwise_and(depthMat, depthMat, mask= mask)

    # Resize the image back to its original size
    masked = cv.resize(masked, None, (0,0), fx = 0.5, fy = 0.5, interpolation=cv.INTER_NEAREST)
    #ShowImage(masked, True)

    digit = fm.GetDigitFromFilename(f)

    # Save the filtered depth image
    # cv.imwrite(outputDir + "/Depth_" + str(digit) + ".tiff", masked)

    



