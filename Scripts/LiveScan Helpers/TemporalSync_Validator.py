# This script checks if the temporal offset of the timestamps from different temporally synced Kinect cameras are in a acceptable ranges
#Drop the TimeStamps_ClientX.txt files of all devices into the same path as this script and then run it to validate the files

import os.path

allTimeStampLists = []
ideaTimeDifferenceus = 160
acceptableTimeDifferenceus = 200

fileBaseName = "Timestamps_Client"
fileType = ".txt"
fileIndex = 0

fileFound = True

while(fileFound):
    if(os.path.isfile(fileBaseName + str(fileIndex) + fileType)):
        fileIndex+= 1

    else:
        fileFound = False

if(fileIndex < 2):
    print("ERROR: There must be at least two Timestamp Logs for comparision!")
    exit(1)

for i in range(0, fileIndex):
    file = open(fileBaseName + str(i) + fileType)
    lines = file.readlines()

    timeStampList = []

    for line in lines:
        splitLine = line.split()
        timeStampList.append(int(splitLine[1]))

    allTimeStampLists.append(timeStampList)


maxFrames = 0

for list in allTimeStampLists:
    if(len(list) > maxFrames):
        maxFrames = len(list)


for i in range(0, maxFrames):
    timeStampsFromOneFrame = []
    for list in allTimeStampLists:
        if i <= (len(list) - 1):
            timeStampsFromOneFrame.append(list[i])
        else:
            timeStampsFromOneFrame.append(-1)

    timeStampsFromOneFrame.sort()
    if(len(timeStampsFromOneFrame) == 0):
        print("Frame: " + str(i) + " ERROR: Frame Contains no timestamps")

    elif(timeStampsFromOneFrame[0] == -1):
        print("Frame: " + str(i) + " ERROR: One or more devices don't have a timestamp for this frame")

    else:
        minTimeStamp = timeStampsFromOneFrame[0]
        maxTimeStamp = timeStampsFromOneFrame[len(timeStampsFromOneFrame ) - 1]

        nsDifference = maxTimeStamp - minTimeStamp
        usDifference = int(nsDifference / 1000)
        msDifference = (usDifference / 1000)

        if usDifference < ((len(allTimeStampLists) - 1) * ideaTimeDifferenceus):
            print("Frame: " + str(i) + " Largest Difference in us between frames: " + str(usDifference) + "(Good)")

        elif usDifference < ((len(allTimeStampLists) - 1) * acceptableTimeDifferenceus):
            print("Frame: " + str(i) + " Largest Difference in us between frames: " + str(usDifference) + "(Ok)")

        elif usDifference > ((len(allTimeStampLists) - 1) * acceptableTimeDifferenceus):
            print("Frame: " + str(i) + " (Too Large) Largest Difference in us between frames: " + str(usDifference) + "(Too large)")








