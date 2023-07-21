import os
from pathlib import Path
import subprocess
import shutil
from sys import exit

#TODOs:
# - How much faster is GPU reconstruction, compared to cpu reconstruction?
# - Implement no cuda, but OpenGL support at least, if it is faster
# - Does using the internal intriniscs of the Kinect improve cam localization?


#Hardcoded paths
base_path = Path(__file__).parent
lib_path = base_path / "lib"
ffmpeg_path = lib_path / "ffmpeg.exe"
colmap_path = lib_path / "colmap"
colmap_bin_path = colmap_path / "colmap.bat"
colmap_vocab_tree_path = colmap_path / "vctree.bin"

#User defined paths
output_path = base_path / "output"
output_path_tmp = output_path / "tmp"


def create_output_folder_structure():
    create_folder(output_path)
    create_folder(output_path_tmp)


def split_video_into_images(video_path):
    
    print("Processing input video")

    # Check path integrity
    exit_if_file_not_exists(ffmpeg_path)
    exit_if_file_not_exists(video_path)

    # Create tmp output folder for images
    bridging_images_path = output_path / "tmp" / "images"
    create_folder(bridging_images_path, True)
    
    # Execute FFMPEG to split video into images
    cmd = str(ffmpeg_path) + " -i " + str(video_path) + " -r 4 " + str(bridging_images_path) + "\output_%04d.png"
    print("Processing video command: " + cmd)
    subprocess.run(cmd) 
    
    return bridging_images_path

def create_sfm_bridge(bridging_images_path):

    print("Creating SfM bridge")

    #Check file integrity
    exit_if_file_not_exists(colmap_bin_path)

    #Create colmap project output dir
    colmap_path = output_path_tmp / "colmap"
    create_folder(colmap_path, True)

    colmap_images_path = colmap_path / "images"
    create_folder(colmap_images_path)

    colmap_sparse_path = colmap_path / "sparse"
    create_folder(colmap_sparse_path)

    colmap_project_db_path = colmap_path / "database.db"


    #Convert to path type, if input is a string
    bridging_images_path = Path(bridging_images_path)

    #Copy all input images into the Colmap project dir
    filecount = 0
    for file in os.listdir(str(bridging_images_path)):
        if file.endswith(".png"):
            try:
                fullpath = bridging_images_path / file
                shutil.copy(str(fullpath), colmap_images_path)
                filecount += 1
            except Exception as e:
                print(e)

    print("Found " + str(filecount) + " images for Structure for Motion reconstruction")

    if(filecount < 1):
        print("Error: No images found for SfM reconstruction!")
        return False

    #Feature extraction
    cmd = str(colmap_bin_path) + " feature_extractor --database_path " + str(colmap_project_db_path) + " --image_path " + str(colmap_images_path) + " --SiftExtraction.use_gpu 0 --ImageReader.single_camera_per_folder 1" 
    print("Running feature extraction: " + cmd)
    subprocess.run(cmd) 

    # Sequential matching
    cmd = str(colmap_bin_path) + " sequential_matcher --database_path " + str(colmap_project_db_path) + " --SiftMatching.use_gpu 0"
    print("Running sequential matching: " + cmd)
    subprocess.run(cmd) 

    #Mapping
    cmd = str(colmap_bin_path) + " mapper --database_path " + str(colmap_project_db_path) + " --image_path " + str(colmap_images_path) + " --output_path " + str(colmap_sparse_path)
    print("Running mapping: " + cmd)
    subprocess.run(cmd) 


    return colmap_path

def register_rgdb_color_images_to_briding_sfm(sfm_bridge_project_path, rgdb_color_images_path):

    sfm_bridge_project_path = Path(sfm_bridge_project_path)
    rgdb_color_images_path = Path(rgdb_color_images_path)

    colmap_project_db = sfm_bridge_project_path / "database.db"
    colmap_project_images = sfm_bridge_project_path / "images"
    input_reconstruction = sfm_bridge_project_path / "sparse/0"
    output_reconstruction = sfm_bridge_project_path / "sparse/output"

    # Check file integrity
    exit_if_file_not_exists(colmap_project_db)
    exit_if_file_not_exists(colmap_bin_path)
    exit_if_file_not_exists(colmap_vocab_tree_path)

    create_folder(output_reconstruction, True)

    image_files = []

    for file in os.listdir(str(rgdb_color_images_path)):
        if file.endswith(".png") or file.endswith(".jpg") or file.endswith(".jpeg"):
            image_files.append(file)

    if(len(image_files) < 1):
        print("Error: No images found for SfM reconstruction!")
        return False

    print("Found " + str(len(image_files)) + " RGBD color images to register on SfM model")

    print("Creating image list and copying files")
    image_list_file = sfm_bridge_project_path / "image_list.txt"
    with open(str(image_list_file), 'w') as f:
        for image in image_files:
            f.write(image)
            f.write('\n')
            image_fullpath = rgdb_color_images_path / image
            shutil.copy(image_fullpath, colmap_project_images)


    #Feature extraction
    cmd = str(colmap_bin_path) + " feature_extractor --database_path " + str(colmap_project_db) + " --image_path " + str(colmap_project_images) + " --image_list_path " + str(image_list_file) + " --SiftExtraction.use_gpu 0"
    print("Running feature extraction: " + cmd)
    subprocess.run(cmd) 

    #Vocabulary Tree matching
    cmd = str(colmap_bin_path) + " vocab_tree_matcher --database_path " + str(colmap_project_db) + " --VocabTreeMatching.vocab_tree_path " + str(colmap_vocab_tree_path) + " --VocabTreeMatching.match_list_path " + str(image_list_file) + " --SiftMatching.use_gpu 0"
    print("Running vocabulary tree matching: " + cmd)
    subprocess.run(cmd) 

    #Mapping
    cmd = str(colmap_bin_path) + " mapper --database_path " + str(colmap_project_db) + " --image_path " + str(colmap_project_images) + " --input_path " + str(input_reconstruction) + " --output_path " + str(output_reconstruction)
    print("Running feature extraction: " + cmd)
    subprocess.run(cmd) 

    return output_reconstruction



def create_folder(folder, delete_if_exists = False):
    if(os.path.exists(folder) and delete_if_exists):
        try:
            shutil.rmtree(folder)
        except Exception as e:
            print(e)
    
    if not (os.path.exists(folder)):
        try:
            os.mkdir(folder)
        except Exception as e:
            print(e)
            return False
    
def exit_if_file_not_exists(filepath):
    if(not os.path.isfile(filepath)):
       exit("Fatal error: Could not find file: " + str(filepath))