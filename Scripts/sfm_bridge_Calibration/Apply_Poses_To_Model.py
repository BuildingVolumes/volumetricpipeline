import os
import pymeshlab
from pathlib import Path
from read_write_model import read_transformation_mat
from cam_poses_by_sfm_bridging import registeredRGBDCam
import rerun as rr
import numpy as np


calibration_Path = "C:\Dev\Volcapture\Experiments\\volumetricpipeline\Scripts\sfm_bridge_Calibration\Tests"
pointcloud_Path = "C:\Dev\Volcapture\Experiments\\volumetricpipeline\Scripts\sfm_bridge_Calibration\Tests"
export_Path = "C:\Dev\Volcapture\Experiments\\volumetricpipeline\Scripts\sfm_bridge_Calibration\Tests"


red = pymeshlab.Color(255, 0, 0, 255)
green = pymeshlab.Color(0, 255, 0, 255)
blue = pymeshlab.Color(0, 0, 255, 255)
yellow = pymeshlab.Color(0, 255, 255, 255)

colors = [red, green, blue, yellow]



matrices = []

for calib_file in os.listdir(str(calibration_Path)):
    if calib_file.endswith(".txt"):
        print(calib_file)
        sfm_matrix, icp_matrix, combined_matrix = read_transformation_mat(os.path.join(calibration_Path, calib_file))

        matrices.append(np.matmul(icp_matrix, sfm_matrix))


pointclouds = []

for pc_file in os.listdir(str(pointcloud_Path)):
    if pc_file.endswith(".ply"):
        print(pc_file)
        pointcloud_fullpath = os.path.join(pointcloud_Path, pc_file)
        pointcloud = pymeshlab.MeshSet()
        pointcloud.load_new_mesh(str(pointcloud_fullpath))
        pointclouds.append(pointcloud)


rr.init("Fuse")
rr.spawn()

fused = pymeshlab.MeshSet()
differencemap = pymeshlab.MeshSet()

for i in range(0, len(pointclouds) - 1):
    
    #Generate transformation matrix for the pointcloud from SfM-pipeline results and apply
    pointclouds[i].set_matrix(transformmatrix = matrices[i])
    pointclouds[i].apply_matrix_freeze()

    pc = pointclouds[i].current_mesh()
    vertices = pc.vertex_matrix()
    colors = pc.vertex_color_matrix()

    rr.log_points("rgbd" + str(i), vertices, colors=colors, radii=0.01)

    fused.add_mesh(pc)
    differencePC = pointclouds[i]
    differencePC.set_color_per_vertex(color1=colors[i])
    differencemap.add_mesh(differencePC.current_mesh())


fused.generate_by_merging_visible_meshes(mergevisible = True, deletelayer = True)
fused.save_current_mesh(os.path.join(export_Path, "Fused.ply"))

differencemap.generate_by_merging_visible_meshes(mergevisible = True, deletelayer = True)
differencemap.save_current_mesh(os.path.join(export_Path, "Difference.ply"))