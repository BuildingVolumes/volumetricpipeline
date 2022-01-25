import os
from PIL import Image #https://pillow.readthedocs.io/en/stable/
import argparse
from BackEnd.contentToJson import contentToJson

def parseCommandLine():
    parser = argparse.ArgumentParser(description="Converts a folder of pictures into another file type")
    parser.add_argument("inDir", metavar="inDir", type=str, help="The folder that contains the pictures you want to convert")
    parser.add_argument("outDir", metavar="outDir", type=str, help="The folder to save the converted files")
    parser.add_argument("type", metavar="type", type=str, help="The file type to convert to")

    args = parser.parse_args()
    return args

def nodeConvertTiff(inDir, outDir, fileType, identifer):
    convertTiff(inDir, outDir, fileType)
    result = {0: inDir}
    contentToJson(identifer, result)

def convertTiff(inDir, outDir, fileType):
    for foldername in os.listdir(inDir):
        
        folder = inDir + foldername + "/"
        print('checking file: ' + folder)
        if os.path.isdir(folder):
            for filename in os.listdir(folder):
            
                if ("." in filename):
                    fileParts = filename.split('.')
                
                    if(fileParts[1] == "tiff"):
                    
                        tiff = Image.open(folder + filename)
                        tiff.save(folder + fileParts[0] + "." + fileType, fileType)
                        #os.remove(folder + filename)
def main():
    arguments = parseCommandLine()

    print("Input Dir is ", arguments.inDir)
    print("Output Dir is ", arguments.outDir)
    print("File Type is ", arguments.type)

    #print(os.listdir(arguments.inDir))
    convertTiff(arguments.inDir, arguments.outDir, arguments.type)

#main()


