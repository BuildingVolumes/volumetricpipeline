import pymeshlab
import os
from pathlib import Path


path_to_sequence = input("Enter the absolute path to the unmerged sequences: ")
path_to_sequence = Path(path_to_sequence)
numCams = int(input("Enter the number of cameras: "))

if not (os.path.exists(path_to_sequence)):
        print("Path does not exist!")
        exit()

path_to_merged_sequence = path_to_sequence / "merged"

if not (os.path.exists(path_to_merged_sequence)):
    try:
        os.mkdir(path_to_merged_sequence)
    except Exception as e:
        print(e)
        exit()


pc_files = []

for file in os.listdir(str(path_to_sequence)):
    if file.endswith(".ply"):
        pc_files.append(file)

if(len(pc_files) % numCams != 0):
     print("Number of cameras not set correctly!")
     exit()

frames = int(len(pc_files) / numCams)
currentFrame = 0

for frame in range(frames):

    ms = pymeshlab.MeshSet()

    print("Merging Frame: " + str(currentFrame))
  
    for pc in pc_files:
            
        name = str(pc).split('.')[0]
        frameNum = str(name).split("_")[0]
        if(int(frameNum) == currentFrame):
            fullpath = path_to_sequence / str(pc)
            ms.load_new_mesh(str(fullpath))
    
    if(len(ms) != numCams):
         print("WARNING: Could not find all cameras for frane " + str(currentFrame))
         continue

    ms.generate_by_merging_visible_meshes()
    ms.save_current_mesh(str(path_to_merged_sequence) + "/" + str(currentFrame) + "_cleaned.ply")
    currentFrame += 1

