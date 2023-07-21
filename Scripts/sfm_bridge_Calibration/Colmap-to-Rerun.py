import rerun as rr
import pymeshlab
import os

from pathlib import Path
from scipy.spatial.transform import Rotation as R

import collections
import numpy as np
from simpleicp import PointCloud, SimpleICP

from read_write_model import Camera, read_model
from sfm_pipeline import *

# A collection of all data we'll be creating for a registered Depth Camera
registeredRGBDCam = collections.namedtuple("registeredRGBDCam", ["id", "name", "meshset", "sfm_matrix", "icp_matrix", "pointcloud_file"])

#Hardcoded paths
base_path = Path(__file__).parent
output_path = base_path / "output"

rgbd_Pointclouds_path = base_path / "input/pointclouds"
rgbd_Color_images_path = base_path / "input/color_images"
video_path = base_path / "input/video/video.mp4"

#Sub paths of user defined paths
intermediate_results_path = output_path / "intermediate_results"


scale_factor = 3.85
visualize = True #Visualize pipeline in rerun
save_intermediate_results = True #Save the pointclouds generated in each step

#Mock intrinsics to show camera in rerun
mock_width = 1920
mock_height = 1080
mock_u_cen = mock_width / 2
mock_v_cen = mock_height / 2
mock_f_len = (mock_height * mock_width) ** 0.5
mock_cam_instrinsics = [[mock_f_len, 0,    mock_u_cen], [0,     mock_f_len, mock_v_cen], [0,     0,     1  ]]


def read_sparse_reconstruction(dataset_path: Path):
    print("Reading sparse COLMAP reconstruction")
    dataset_path = Path(dataset_path)
    cameras, images, points3D = read_model(dataset_path, ext=".bin")

    #Seperate the rgbd camera poses from the SfM process into it's own dictionary
    rgbd_images = []
    rgbdcam_keys = []

    for image in images:
        if "Client" in images[image].name:
            rgbd_images.append(images[image])
            rgbdcam_keys.append(image)

    for key in rgbdcam_keys:
        del images[key]

    return cameras, images, rgbd_images, points3D

def get_sparse_reconstruction_mesh(cameras: dict, images: dict, rgbd_images: dict, points3D: dict, filter_output: bool) -> None:
    
    print("Getting sparse reconstruction pointcloud from SfM process")

    # Filter out noisy points, important for ICP refinement later on
    if filter_output:
        points3D = {id: point for id, point in points3D.items() if point.rgb.any() and len(point.image_ids) > 4}

    #Get all points into one array (or: cloud)
    allPoints = []
    allColors = []

    for point in points3D:
        allPoints.append(points3D[point].xyz)
        allColors.append(points3D[point].rgb)

    # Create reference point cloud from the SfM process in pymeshlab
    # We color all points green so that we can visualize it better
    referencePC = pymeshlab.MeshSet()
    newMesh = pymeshlab.Mesh(vertex_matrix=allPoints)
    referencePC.add_mesh(newMesh)
    greencolor = pymeshlab.Color(0, 255, 0, 255)
    referencePC.set_color_per_vertex(color1=greencolor)

    if(visualize):
        rr.log_view_coordinates("/", up="-Y", timeless=True)
        rr.log_points("points_sfm", allPoints, radii=0.01)

        # Iterate through images (video frames) logging data related to each frame.
        for image in sorted(images.values(), key=lambda im: im.name):  # type: ignore[no-any-return]

            quat_xyzw = image.qvec[[1, 2, 3, 0]]  # COLMAP uses wxyz quaternions

            # COLMAP's camera transform is "camera from world"
            rr.log_transform3d(
                "phonecamera", rr.TranslationRotationScale3D(image.tvec, rr.Quaternion(xyzw=quat_xyzw)), from_parent=True
            )
            rr.log_view_coordinates("phonecamera", xyz="RDF")  # X=Right, Y=Down, Z=Forward

            # Log camera intrinsics
            rr.log_pinhole(
                "phonecamera/image",
                child_from_parent=mock_cam_instrinsics,
                width=mock_width,
                height=mock_height,
            )
    
    if(save_intermediate_results):
        create_folder(intermediate_results_path, True)
        referencePC.save_current_mesh(os.path.join(intermediate_results_path, "referenceMesh.ply"))

    return referencePC

 
def generate_transformation_matrix(tVec, QVec, scale):
     
    # Get Rotation from Quaternion
    r = R.from_quat(QVec)

    #Invert, because the rotations describes the rotation from local to world, we need world to local
    r = r.inv()

    #Get as 3x3 Rotation matrix
    r = r.as_matrix()

    #Translate into homogeneous transformation matrix
    rot_matrix = np.array([[r[0][0], r[0][1], r[0][2], 0],
                           [r[1][0], r[1][1], r[1][2], 0],
                           [r[2][0], r[2][1], r[2][2], 0],
                           [0, 0, 0, 1]])

    #Translate translation vector into homogeneous transformation matrix and invert aswell
    trans_matrix = np.array([[1, 0, 0, tVec[0] * -1],
                           [0, 1, 0, tVec[1] * -1],
                           [0, 0, 1, tVec[2] * -1],
                           [0, 0, 0, 1]])

    #Translate uniform scale into homogeneous transformation matrix aswell
    s = scale
    scale_matrix = np.array([[s, 0, 0, 0],
                           [0, s, 0, 0],
                           [0, 0, s, 0],
                           [0, 0, 0, 1]])

    #Combine all transformations into one matrix
    rt_matrix = np.matmul(rot_matrix, trans_matrix)
    trs_matrix = np.matmul(rt_matrix, scale_matrix)

    return trs_matrix

def register_pointcloud_from_sfm_poses(rgbdcam_poses):
    
    registeredCams = {}

    print("Registering RGBD pointclouds")

    for rgbdcam in rgbdcam_poses:

        pointcloud_name = rgbdcam.name.split('.')[0]
        pointcloud_filename = pointcloud_name + ".ply"

        print(pointcloud_filename)

        #Get rgbd camera pose from SfM Process
        trans_xyz = rgbdcam.tvec
        quat_wxyz = rgbdcam.qvec
        quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]] #switch Quaternion format into the more common xyzw format 

        #Load pointcloud from rgbd camera from disk
        pointcloud_fullpath = os.path.join(rgbd_Pointclouds_path, pointcloud_filename)
        pointcloud = pymeshlab.MeshSet()
        pointcloud.load_new_mesh(str(pointcloud_fullpath))

        #Generate transformation matrix for the pointcloud from SfM-pipeline results and apply
        trs_matrix = generate_transformation_matrix(tVec=trans_xyz, QVec=quat_xyzw, scale=scale_factor)
        pointcloud.set_matrix(transformmatrix = trs_matrix)
        pointcloud.apply_matrix_freeze()

        #Save results into our dictionary
        registeredCams[rgbdcam.id] = registeredRGBDCam(id=rgbdcam.id, name=pointcloud_name, meshset=pointcloud, sfm_matrix=trs_matrix, icp_matrix=None, pointcloud_file=str(pointcloud_filename))

        if(visualize):
            #Visualize the camera pose in rerun
            rr.log_transform3d(pointcloud_name, rr.TranslationRotationScale3D(trans_xyz, rr.Quaternion(xyzw=quat_xyzw)), from_parent=True)
            rr.log_view_coordinates(pointcloud_name, xyz="RDF")  # X=Right, Y=Down, Z=Forward
            rr.log_pinhole(pointcloud_name + "/image", child_from_parent = mock_cam_instrinsics, width = mock_width, height = mock_height)

            pc = pointcloud.current_mesh()
            vertices = pc.vertex_matrix()
            colors = pc.vertex_color_matrix()
            rr.log_points("rgbd" + pointcloud_name, vertices, colors=colors, radii=0.01)

        if(save_intermediate_results):
            pointcloud.save_current_mesh(os.path.join(intermediate_results_path, ("sfm_pose_" + pointcloud_name + ".ply")))   



    return registeredCams


def refine_poses_ICP(referenceMesh : pymeshlab.MeshSet, registeredCams : dict):

    refinedCams = {}
    
    print("Refining RGBD poses with RGB")

    #The sparse pointcloud from the SfM process is used as reference for the ICP process, which we calibrate all pointclouds against
    verticesRef = referenceMesh.current_mesh().vertex_matrix()
    pc_fix = PointCloud(verticesRef, columns=["x", "y", "z"])
    
    for cam in registeredCams:

        registeredCam = registeredCams[cam]

        print("Refining pose for RGBD camera: " + registeredCam.name)

        ICP_pointcloud = registeredCam.meshset

        vertices = ICP_pointcloud.current_mesh().vertex_matrix()
        pc_mov = PointCloud(vertices, columns=["x", "y", "z"])

        #Use the SimpleICP library to calculate our ICP refinement matrix
        icp = SimpleICP()
        icp.add_point_clouds(pc_fix, pc_mov)
        icp_matrix, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1, max_iterations=30)

        #Apply refinement pose to our pointcloud
        ICP_pointcloud.set_matrix(transformmatrix = icp_matrix)
        ICP_pointcloud.apply_matrix_freeze()

        refinedCams[cam] = registeredRGBDCam(id=cam, name=registeredCam.name, meshset=ICP_pointcloud, sfm_matrix=registeredCam.sfm_matrix, icp_matrix=icp_matrix, pointcloud_file=registeredCam.pointcloud_file)

        if(visualize):
            icp_pc = ICP_pointcloud.current_mesh()
            vertices = icp_pc.vertex_matrix()
            colors = icp_pc.vertex_color_matrix()
            rr.log_points("rgbd" + registeredCam.name, vertices, colors=colors, radii=0.01)

        if(save_intermediate_results):
            ICP_pointcloud.save_current_mesh(os.path.join(intermediate_results_path, ("sfm_icp_pose_" + registeredCam.name + ".ply")))

    
    return refinedCams


def main():

    run_image_creation = True
    run_bridging_sfm = True
    register_rgbd_images_to_SfM = True

    visualize = True

    create_output_folder_structure()


    sfm_image_path = "C:\Dev\Volcapture\Experiments\volumetricpipeline\Scripts\sfm_bridge_Calibration\output\\tmp\images"
    if(run_image_creation):
        sfm_image_path = split_video_into_images(video_path)

    bridging_colmap_path = "C:\Dev\Volcapture\Experiments\\volumetricpipeline\Scripts\sfm_bridge_Calibration\output\\tmp\colmap"
    if(run_bridging_sfm):
        bridging_colmap_path = create_sfm_bridge(sfm_image_path)

    complete_reconstruction = "C:\Dev\Volcapture\Experiments\\volumetricpipeline\Scripts\sfm_bridge_Calibration\output\\tmp\colmap\sparse\output"
    if(register_rgbd_images_to_SfM):
        complete_reconstruction = register_rgdb_color_images_to_briding_sfm(bridging_colmap_path, rgbd_Color_images_path)

    if(visualize):
        rr.init("Colmap to Rerun")
        rr.spawn()

    #Read colmap reconstruction
    cameras, images, rgbd_images, points3D = read_sparse_reconstruction(complete_reconstruction)


    # Get the sparse pointcloud from the SfM, which we'll use as a reference/truth
    referenceMesh = get_sparse_reconstruction_mesh(cameras, images, rgbd_images, points3D, True)
    
    # Read the unregistered pointclouds from 

    registered_RGBD_cams = register_pointcloud_from_sfm_poses(rgbd_images)
    refined_RGBD_cams = refine_poses_ICP(referenceMesh, registered_RGBD_cams)

    #reconstructedMesh.generate_by_merging_visible_meshes(mergevisible = True, deletelayer = True)
    #reconstructedMesh.save_current_mesh(os.path.join(export_path, "reconstruedMesh.ply"))

    #reconstructedMesh.add_mesh(referenceMesh.current_mesh())

    #reconstructedMesh.generate_by_merging_visible_meshes(mergevisible = True, deletelayer = True)
    #reconstructedMesh.save_current_mesh(os.path.join(export_path, "reconstruedMeshWithReference.ply"))

if __name__ == '__main__':
    main()