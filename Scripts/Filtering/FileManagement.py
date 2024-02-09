import os

def GetFiles(path, extension):
    # Check if path exists
    if(not os.path.exists(path)):
        print("Path does not exist")
        return None

    #Return list of files with the given extension
    files = os.listdir(path)
    files = [f for f in files if f.endswith(extension)]
    return files

def SortFileList(files):
    files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
    return files

def CreateOutputDir(path, foldername):

    if (os.path.exists(path + foldername)):
        return path + foldername
    else:
        os.makedirs(path + foldername)
        return path + foldername

def GetDigitFromFilename(filename):
    digit = filename.split(".")[0]
    split_digit = digit.split("_")
    digit = split_digit[len(split_digit) - 1]
    return digit