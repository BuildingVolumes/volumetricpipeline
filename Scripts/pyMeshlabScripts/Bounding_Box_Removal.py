import pymeshlab
import os
from pathlib import Path

path_to_sequence = "D:\\Test_BB_clean\\Zeigefinger"
path_to_sequence = Path(path_to_sequence)
path_to_cleaned_sequence = path_to_sequence / "cleaned"

if not (os.path.exists(path_to_cleaned_sequence)):
    try:
        os.mkdir(path_to_cleaned_sequence)
    except Exception as e:
        print(e)
        exit()



pc_files = []

for file in os.listdir(str(path_to_sequence)):
    if file.endswith(".ply"):
        pc_files.append(file)


for pc in pc_files:

    name = str(pc).split('.')[0]
    fullpath = path_to_sequence / str(pc)

    print("Cleaning: " + name)

    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(fullpath))



    bbMaxX = 0.118043
    bbMinX = -0.050810
    bbMaxY = -0.022260
    bbMinY = -0.055178
    bbMaxZ = 0.057527
    bbMinZ = -0.008243

    cleancmd = "(" + "x > " + str(bbMinX) + " && " + "x < " + str(bbMaxX) + " && " + "y > " + str(bbMinY) + " && " + "y < " + str(bbMaxY) + " && " + "z > " + str(bbMinZ) + " && " + "z < " + str(bbMaxZ) + ")"
    print(cleancmd)
    ms.compute_selection_by_condition_per_vertex(condselect = cleancmd)
    ms.meshing_remove_selected_vertices()


    bbMaxX = -0.037340
    bbMinX = -0.121600
    bbMaxY = 0.013389
    bbMinY = -0.055178
    bbMaxZ = 0.066648
    bbMinZ = -0.020734

    cleancmd = "(" + "x > " + str(bbMinX) + " && " + "x < " + str(bbMaxX) + " && " + "y > " + str(bbMinY) + " && " + "y < " + str(bbMaxY) + " && " + "z > " + str(bbMinZ) + " && " + "z < " + str(bbMaxZ) + ")"
    print(cleancmd)
    ms.compute_selection_by_condition_per_vertex(condselect = cleancmd)
    ms.meshing_remove_selected_vertices()

    bbMaxX = 0.013000
    bbMinX = -0.050138
    bbMaxY = 0.005516
    bbMinY = -0.027741
    bbMaxZ = 0.049739
    bbMinZ = -0.000723

    cleancmd = "(" + "x > " + str(bbMinX) + " && " + "x < " + str(bbMaxX) + " && " + "y > " + str(bbMinY) + " && " + "y < " + str(bbMaxY) + " && " + "z > " + str(bbMinZ) + " && " + "z < " + str(bbMaxZ) + ")"
    print(cleancmd)
    ms.compute_selection_by_condition_per_vertex(condselect = cleancmd)
    ms.meshing_remove_selected_vertices()

    bbMaxX = 0.110796
    bbMinX = 0.068380
    bbMaxY = 0.005516
    bbMinY = -0.027741
    bbMaxZ = 0.058317
    bbMinZ = 0

    cleancmd = "(" + "x > " + str(bbMinX) + " && " + "x < " + str(bbMaxX) + " && " + "y > " + str(bbMinY) + " && " + "y < " + str(bbMaxY) + " && " + "z > " + str(bbMinZ) + " && " + "z < " + str(bbMaxZ) + ")"
    print(cleancmd)
    ms.compute_selection_by_condition_per_vertex(condselect = cleancmd)
    ms.meshing_remove_selected_vertices()
    
    ms.save_current_mesh(str(path_to_cleaned_sequence) + "/" + name + "_cleaned.ply")


