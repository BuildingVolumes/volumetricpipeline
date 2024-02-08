from read_write_model import read_transformation_mat, write_transformation_mat_LiveScan
from cam_poses_by_sfm_bridging import registeredRGBDCam, create_folder
from pathlib import Path
import os
import numpy as np



path = Path("output/calibration")
livescan_calib_path = path / "livescan"


create_folder(livescan_calib_path, True)


calib_files = []

for calib_file in os.listdir(str(path)):
    if calib_file.endswith(".txt"):
        calib_files.append(calib_file)

filecount = 1

for calib_file in calib_files:
    
    fullpath = path / calib_file
    #filename = str(calib_file).split(".")[0]
    filename = "calibration_000000000000" + str(filecount)
    filecount += 1
    sfm_matrix, icp_matrix, combined_matrix = read_transformation_mat(fullpath)

    identity_matrix = np.zeros(shape=(4,4))
    identity_matrix[0][0] = 1
    identity_matrix[1][1] = 1
    identity_matrix[2][2] = 1
    identity_matrix[3][3] = 1

    # Change coordinate system from Colmap to OpenGL
    # Invert Y Axis
    sfm_matrix[1][0] = sfm_matrix[1][0] * -1
    sfm_matrix[1][1] = sfm_matrix[1][1] * -1
    sfm_matrix[1][2] = sfm_matrix[1][2] * -1
    sfm_matrix[1][3] = sfm_matrix[1][3] * -1

    icp_matrix[1][0] = icp_matrix[1][0] * -1
    icp_matrix[1][1] = icp_matrix[1][1] * -1
    icp_matrix[1][2] = icp_matrix[1][2] * -1
    icp_matrix[1][3] = icp_matrix[1][3] * -1

    # Invert Z Axis
    sfm_matrix[2][0] = sfm_matrix[2][0] * -1
    sfm_matrix[2][1] = sfm_matrix[2][1] * -1
    sfm_matrix[2][2] = sfm_matrix[2][2] * -1
    sfm_matrix[2][3] = sfm_matrix[2][3] * -1

    icp_matrix[2][0] = icp_matrix[2][0] * -1
    icp_matrix[2][1] = icp_matrix[2][1] * -1
    icp_matrix[2][2] = icp_matrix[2][2] * -1
    icp_matrix[2][3] = icp_matrix[2][3] * -1

    icp_matrix = linalg.inv(icp_matrix)

    print("Writing Matrix: " + str(filename))
    write_transformation_mat_LiveScan(livescan_calib_path, filename, sfm_matrix, identity_matrix)

