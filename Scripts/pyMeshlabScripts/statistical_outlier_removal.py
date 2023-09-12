import pymeshlab
import os
from pathlib import Path

path_to_sequence = "C:\\Users\\chris\Downloads\\wetransfer_thumbgs_2023-08-30_0851"
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

    ms.compute_selection_point_cloud_outliers(knearest = 128)
    ms.meshing_remove_selected_vertices()
    
    ms.save_current_mesh(str(path_to_cleaned_sequence) + "/" + name + "_cleaned.ply")
    


