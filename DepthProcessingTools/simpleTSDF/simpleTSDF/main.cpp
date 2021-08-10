#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>

#include "meshview/meshview.hpp"
#include "meshview/meshview_imgui.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "TSDFVolume.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include <k4a/k4a.h>

using namespace meshview;

TSDFVolume *theVolume;
Viewer viewer;
std::map<int, Eigen::Matrix4d> extrinsics;
std::map<int, Eigen::Matrix3d> intrinsics;
std::map<int, k4a_calibration_t> k4aCalibrations;
int VOXRES = 256;
int VOXSMOOTH = 1;

void AddVolumeToViewer(TSDFVolume *v) {
   // theVolume->Smooth(VOXSMOOTH);
    double isolevel = 1.0f/theVolume->res[0]/2;
    std::vector<TRIANGLE> tris;
    int n = v->PolygoniseMC(isolevel, tris);
    // * Triangle mesh: single color
    Points verts(3 * n, 3);
    Points colors(3 * n, 3);
    for (int i = 0, ind=0; i < n; i++, ind+=3)
    {
        verts(ind+0, 0) = tris[i].p[0][0]; 
        verts(ind+0, 1) = tris[i].p[0][1];
        verts(ind+0, 2) = tris[i].p[0][2];

        verts(ind + 1, 0) = tris[i].p[1][0];
        verts(ind + 1, 1) = tris[i].p[1][1];
        verts(ind + 1, 2) = tris[i].p[1][2];
        
        verts(ind + 2, 0) = tris[i].p[2][0];
        verts(ind + 2, 1) = tris[i].p[2][1];
        verts(ind + 2, 2) = tris[i].p[2][2];

        colors(ind + 0, 0) = tris[i].c[0];
        colors(ind + 0, 1) = tris[i].c[1];
        colors(ind + 0, 2) = tris[i].c[2];
  
        colors(ind + 1, 0) = tris[i].c[0];
        colors(ind + 1, 1) = tris[i].c[1];
        colors(ind + 1, 2) = tris[i].c[2];
        
        colors(ind + 2, 0) = tris[i].c[0];
        colors(ind + 2, 1) = tris[i].c[1];
        colors(ind + 2, 2) = tris[i].c[2];
        
    }

    auto& pyra = viewer
        .add_mesh(verts,
                  /* Triangle indices (unsigned fx3) here; pass
                     empty to use: 0 1 2, 3 4 5 etc */
                  Triangles(),
                    colors
        )
        .translate(Vector3f(0,0,0))
        .set_shininess(32.f);

    /*
    for (int i = 0; i < tris.size(); i++) {
        verts << tris[i].p[0][0], tris[i].p[0][1], tris[i].p[0][2];
    }
    viewer.add_mesh(verts, Triangles(), 0, 1, 0)
        .translate(Vector3f(0.f, 0.f, 0.f))
        .set_shininess(32.f)
        ;
        */


    //for (int k = 0; k < v->res[2]; k++) {
    //    for (int j = 0; j < v->res[1]; j++) {
    //        for (int i = 0; i < v->res[0]; i++) {
    //            Voxel vx = v->get(i, j, k);
    //            if (vx.flag == VOXEL_UNSEEN) {
    //                float r, g, b;
    //                r = vx.r/255.f;
    //                g = 1;// vx.g / 255.f;
    //                b = vx.b/255.f;

    //                viewer.add_cube(Eigen::Vector3f(vx.c(0), vx.c(1), vx.c(2)), v->vSize[0], Eigen::Vector3f(r, g, b));
    //            }
    //            if (vx.flag == VOXEL_FULL) {
    //                float r, g, b;
    //                r = vx.r / 255.f;
    //                g = vx.g / 255.f;
    //                b = vx.b / 255.f;

    //                viewer.add_cube(Eigen::Vector3f(vx.c(0), vx.c(1), vx.c(2)), v->vSize[0], Eigen::Vector3f(r, g, b));
    //            }

    //        }
    //    }
    //}

}

void LoadIntrinsics(std::string fname, int cameraNum) {
    k4a_calibration_t calibration;
    std::ifstream intrinsic_file = std::ifstream(fname);
    std::string file_contents;

    intrinsic_file.seekg(0, std::ios::end);
    size_t file_length = intrinsic_file.tellg();
    intrinsic_file.seekg(0, std::ios::beg);

    file_contents.resize(file_length);
    intrinsic_file.read(&file_contents[0], file_length);

    intrinsic_file.close();

    k4a_result_t res = k4a_calibration_get_from_raw(&file_contents[0], file_contents.size() + 1, k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED, k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P, &calibration);
    if (res == k4a_result_t::K4A_RESULT_FAILED)
    {
        std::cout << "ERROR: FAILED TO READ INTRINSICS JSON!" << std::endl;
    }

    auto params = calibration.color_camera_calibration.intrinsics.parameters;

    Eigen::Matrix3d intrinsic_mat = Eigen::Matrix3d::Identity();
    intrinsic_mat(0, 0) = params.param.fx;
    intrinsic_mat(1, 1) = params.param.fy;
    intrinsic_mat(0, 2) = params.param.cx;
    intrinsic_mat(1, 2) = params.param.cy;
    std::cout << intrinsic_mat << std::endl;
    intrinsics[cameraNum] = intrinsic_mat;
    k4aCalibrations[cameraNum] = calibration;

}

void LoadExtrinsics(std::string fname) {
    std::fstream extrinsics_filestream;
    extrinsics_filestream.open(fname);
    int count = 0;
    int camNum = 0;
    int modCount;

    while (!extrinsics_filestream.eof()) {
        std::vector<std::string> extrinsics_string;
        std::string parser = "";
        if (!std::getline(extrinsics_filestream, parser)) break;
        std::cout << parser << std::endl;
        std::stringstream str(parser);
        int a, b, c;
        str >> a >> b >> c;
        std::cout << "cam:" << c << std::endl;
        int theCam = c;
        Eigen::Matrix4d theMat;
        for (int i = 0; i < 4; ++i)
        {
            float aa, bb, cc, dd;
            std::getline(extrinsics_filestream, parser);
            std::stringstream ss(parser);
            ss >> aa >> bb >> cc >> dd;
            theMat(i, 0) = aa;
            theMat(i, 1) = bb;
            theMat(i, 2) = cc;
            theMat(i, 3) = dd;

        }
        extrinsics[theCam] = theMat;// .inverse();
        count++;
    }
}

float GetAllDepth(std::vector<Eigen::Vector3d>& pts, cv::Mat &imDepth, std::vector<float> &depths) {
    float sum = 0.f;
    int num=0;
    float min = 1000;
    float max = -10000;
    for (int i = 0; i < pts.size(); i++) {
        int u, v;
        u = (int)pts[i](0);
        v = (int)pts[i](1);
        float uvz = pts[i](2);
        if (u > 0 && u < imDepth.cols && v > 0 && v < imDepth.rows) { // is the projected voxel center in the image?
            ushort depth = imDepth.at<ushort>(v, u); // get depth image value            
            float depthMeasurement = (float)depth / 1000.f;
            depths.push_back(depthMeasurement);
            //if (depthMeasurement < 2) {
                sum += depthMeasurement;
                num++;
            //}
                min = fminf(min, depthMeasurement);
                max = fmaxf(max, depthMeasurement);
        }
    }
    return min;
//    if (num > 0) return (sum / (float)num);
  //  else return 1;
}

Eigen::Vector3d ProjectPoint(Eigen::Vector3d &p, Eigen::Matrix3d &intrin, Eigen::Matrix4d &exInv, Eigen::Vector4d &cc) {
    /* c is center in world coordinates */
                    // convert to camera coordinates
    cc = exInv * Eigen::Vector4d(p(0), p(1), p(2), 1);

    /* project the center of the voxel onto the images */
    Eigen::Vector3d pp;
    float invz = 1.0f / cc(2);
    float fx, fy, cx, cy;
    fx = intrin(0, 0);
    fy = intrin(1, 1);
    cx = intrin(0, 2);
    cy = intrin(1, 2);
    pp(0) = cc(0) * fx * invz + cx;  // u
    pp(1) = cc(1) * fy * invz + cy;  // v
    pp(2) = cc(2); // store the Z value here (depth from camera of this point)

    return pp;
}
void ProjectAllPoints(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix3d& intrin, Eigen::Matrix4d& exInv, std::vector<Eigen::Vector3d>& ptsOut, std::vector<Eigen::Vector4d>& ccOut) {
    for (int i = 0; i < pts.size(); i++) {
        Eigen::Vector4d cc;
        Eigen::Vector3d v = ProjectPoint(pts[i], intrin, exInv, cc);
        ptsOut.push_back(v);
        ccOut.push_back(cc);
    }
}
void CarveWithSilhouette(TSDFVolume *vol, Eigen::Matrix3d &in, Eigen::Matrix4d &ex, cv::Mat &imRGB, cv::Mat &imMATTE, cv::Mat &imDepth, k4a_image_t &k4a_pointcloud) {
    /* ok, so the standard simplest way  */
    /* for each voxel */
    //v->makeSphereSDF(0.75f);
    //return;
    std::cout << "extrinsics:" << std::endl;
    std::cout << ex << std::endl;

    /* extrinsics passed in convert from camera to wold
       - i.e. multiplying by camera origin (0,0,0) gives the position of the camera in the world to draw
       - we need the transform to convert world coordinates (voxel coords) to be relative to the camera
       -  
    */

    auto ExInv = ex;// .inverse();
    auto R = ex.block<3, 3>(0, 0);
    auto T = ex.block<3, 1>(0, 3);
    
    auto Rt = R.transpose();
    auto T2 = -Rt * T;
    ExInv.block<3, 3>(0, 0) = Rt;
    ExInv.block<3, 1>(0, 3) = T2;

    std::cout << "Ex:" << ex << std::endl;
    std::cout << "R:" << R << std::endl;
    std::cout << "T:" << T << std::endl;
    std::cout << "ExInv:" << ExInv << std::endl;
    float trunc_margin = vol->vSize[0]*5;
    for (int k = 0; k < vol->res[2]; k++) {
        for (int j = 0; j < vol->res[1]; j++) {
            for (int i = 0; i < vol->res[0]; i++) {
                Voxel& vx = vol->get(i, j, k);
                // if the current voxel is already carved in any image then it should be empty for sure
               // if (vx.flag != VOXEL_EMPTY)
                {
                    Eigen::Vector3d c;
                    vol->GetVoxelCoordsFromIndex(i, j, k, c);
                    std::vector<Eigen::Vector3d> allcorners;
                    // project voxel center on to image
                    Eigen::Vector4d pRotExInv;
                    Eigen::Vector3d proj = ProjectPoint(c, in, ExInv, pRotExInv);

                    //// project all corners of the voxel
                    //vol->GetVoxelCornersFromIndex(i, j, k, allcorners);
                    //std::vector<Eigen::Vector3d> ptsOut;
                    //std::vector<Eigen::Vector4d> ccOut;
                    //ProjectAllPoints(allcorners, in, ExInv, ptsOut, ccOut);

                    //std::vector<float> allDepth;
                    //float avgDepth = GetAllDepth(ptsOut, imDepth, allDepth);

                    /* now look in image */
                    int u, v;
                    u = (int)proj(0);
                    v = (int)proj(1);
                    float uvz = proj(2); // this is the depth of the voxel from the camera

                    if (u > 0 && u < imRGB.cols && v > 0 && v < imRGB.rows ) { // is the projected voxel center in the image?
                        cv::Vec3b m   = imMATTE.at<cv::Vec3b>(v,u);  // get matte pixel value
                        cv::Vec3b col = imRGB.at<cv::Vec3b>(v, u); // get colour
                        ushort depth  = imDepth.at<ushort>(v, u); // get depth image value            
                       
                        int16_t* pcData = (int16_t*)k4a_image_get_buffer(k4a_pointcloud);
                        int pcIndex = 3 * (u + v * imRGB.cols);
                        float PCX = (float)(pcData[pcIndex + 0]) /1000.f;
                        float PCY = (float)(pcData[pcIndex + 1]) / 1000.f;
                        float PCZ = (float)(pcData[pcIndex + 2]) / 1000.f;

                        float depthMeasurement = PCZ;// (float)depth / 1000.f; // convert to meters
                        int matte = (int)m[0];
                        if (matte > 200 && depthMeasurement > 0 && depthMeasurement < 3) // does it have a depth value and is it in the foreground?
                        {                          
                           float voxelDepthProjected = uvz;
                           float distFromVoxelToSurfaceSample =  depthMeasurement - voxelDepthProjected;
                           
                          
                           // std::cout << "depthMeasurement:" << depthMeasurement << " (u,v):"<<"("<<u<<","<<v<<"), ushort:"<< depth<<" m:"<<matte<<std::endl;
                           if (distFromVoxelToSurfaceSample > -trunc_margin)
                           {
                              // std::cout << "distsdf:" << distsdf << std::endl;
                               float sdf = fminf(1.f, distFromVoxelToSurfaceSample / trunc_margin);
                               float oldweight = vx.weight;
                               float newweight = oldweight + 1;
                               float weightSum = oldweight + newweight;

                               float d_old = vx.sdf;
                               float d_new = sdf;
                               float d =(d_old * oldweight + d_new) / newweight;
                             //  if(oldweight) 
                               //std::cout << "d_old:" << d_old << " d_new:" << d_new << " d:"<<d<< " distfromVox:"<< distFromVoxelToSurfaceSample << " depthf:"<< depthMeasurement<<std::endl;
                               // std::cout << "d:" << d << std::endl;

                               vx.sdf = d;// (d < -1) ? -1 : (d > 1) ? 1 : d;// d_old + (1.0 / weightSum) * (d_new - d_old);// fmin(d_old, d_new);// d;
                               vx.weight = newweight;

                               vx.r = (oldweight * vx.r + newweight * col[2]) / weightSum;
                               vx.g = (oldweight * vx.g + newweight * col[1]) / weightSum;
                               vx.b = (oldweight * vx.b + newweight * col[0]) / weightSum;
                               vx.flag = VOXEL_FULL;
                           }
                        }
                        //else {
                        //    // the voxel projected to a pixel that didn't have a valid depth OR matte 
                        //    vx.weight = 0;
                        //    vx.sdf = trunc_margin;
                        //    vx.flag = VOXEL_EMPTY;  // carve voxel
                        //}
                    }
                }
            }
        }
    }
  
}

void TransformDepth(cv::Mat old_depth, cv::Mat new_depth, k4a_calibration_t& calibration, k4a_image_t &k4a_pointcloud) {
    k4a_image_t k4a_transformed_depth = nullptr;
    k4a_image_t k4a_depth = nullptr;
//    k4a_image_t k4a_pointcloud = nullptr;
    
    int oldStride = old_depth.step[0];
    if (K4A_RESULT_SUCCEEDED !=
        k4a_image_create_from_buffer(k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, old_depth.cols, old_depth.rows, oldStride,
        old_depth.data, old_depth.step[0]*old_depth.rows, nullptr, nullptr, &k4a_depth))
    {
        std::cout << "Transform Depth error: failed to create k4a image" << std::endl;
        //ErrorLogger::LOG_ERROR("Failed to create a source depth image at " + std::to_string(_timestamp) + ".", true);
    }
    
    int newStride = new_depth.step[0];
    int cw = calibration.depth_camera_calibration.resolution_width;
    int ch = calibration.depth_camera_calibration.resolution_height;
    int ccolw = calibration.color_camera_calibration.resolution_width;
    int ccolh = calibration.color_camera_calibration.resolution_height;
    int old_step1 = old_depth.step1(0);
    int old_nc = old_depth.channels();

    if (K4A_RESULT_SUCCEEDED !=
        k4a_image_create_from_buffer(k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, new_depth.cols, new_depth.rows, newStride,
        new_depth.data, new_depth.step[0]*new_depth.rows, nullptr, nullptr, &k4a_transformed_depth))
    {
        //ErrorLogger::LOG_ERROR("Failed to create a destination depth image at " + std::to_string(_timestamp) + ".", true);
    }

    k4a_image_create_from_buffer(
        K4A_IMAGE_FORMAT_DEPTH16, new_depth.cols, new_depth.rows,
        new_depth.step[0], new_depth.data,
        new_depth.step[0]*new_depth.rows, NULL, NULL,
        &k4a_transformed_depth);
    k4a_transformation_t transform = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(
        transform, k4a_depth, k4a_transformed_depth)) 
    {
        std::cout << "error transforming depth to rgb" << std::endl;
        //ErrorLogger::LOG_ERROR("Failed to transform depth frame to color frame at " + std::to_string(_timestamp) + ".", true);
    }
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, new_depth.cols, new_depth.rows, new_depth.cols * 3 * (int)sizeof(int16_t), &k4a_pointcloud);
    k4a_transformation_depth_image_to_point_cloud(transform, k4a_transformed_depth, K4A_CALIBRATION_TYPE_COLOR, k4a_pointcloud);

    k4a_image_release(k4a_depth);
    k4a_image_release(k4a_transformed_depth);
}

int main(int argc, char** argv) {

    Eigen::Vector3d theCenter(0, 0, 0);
    float sz =2;
    Eigen::Vector3d theSize(sz,sz,sz);
    int res = VOXRES;
    theVolume = new TSDFVolume(res,res,res, theCenter, theSize);
    theVolume->SetAllVoxels(VOXEL_UNSEEN, VOXEL_MAXDIST, 0);
    theVolume->ComputeAllVoxelCenters();   

    viewer.draw_axes = true;
    viewer.light_pos = Vector3f(3,3,0);
    // Adjust camera
    viewer.camera.dist_to_center = 5.f;
   
    /* carve */
    // load in a matte, rgb, and depth image
    std::string path = "C:\\Users\\hogue\\Desktop\\DATA\\July20-calib_3\\";

    std::string fnameExtrinsics = path+"Extrinsics_Open3D.log";

    std::string fnameRGB0 = "client_0\\Color_130.jpg";
    std::string fnameMATTE0 = "client_0\\Color_130.matte.png";
    std::string fnameDEPTH0 = "client_0\\Depth_130.tiff";// 
    std::string fnameIntrinsics0 = "client_0\\Intrinsics_Calib_0.json";
    int camID0 = 0;

    std::string fnameRGB1 = "client_1\\Color_130.jpg";
    std::string fnameMATTE1 = "client_1\\Color_130.matte.png";
    std::string fnameDEPTH1 = "client_1\\Depth_130.tiff";// 
    std::string fnameIntrinsics1 = "client_1\\Intrinsics_Calib_1.json";
    int camID1 = 1;

    std::string fnameRGB2 = "client_2\\Color_130.jpg";
    std::string fnameMATTE2 = "client_2\\Color_130.matte.png";
    std::string fnameDEPTH2 = "client_2\\Depth_130.tiff";// 
    std::string fnameIntrinsics2 = "client_2\\Intrinsics_Calib_2.json";
    int camID2 = 2;

    std::string fnameRGB3 = "client_3\\Color_130.jpg";
    std::string fnameMATTE3 = "client_3\\Color_130.matte.png";
    std::string fnameDEPTH3 = "client_3\\Depth_130.tiff";// 
    std::string fnameIntrinsics3 = "client_3\\Intrinsics_Calib_3.json";
    int camID3 = 3;

    std::string fnameRGB4 = "client_4\\Color_130.jpg";
    std::string fnameMATTE4 = "client_4\\Color_130.matte.png";
    std::string fnameDEPTH4 = "client_4\\Depth_130.tiff";// 
    std::string fnameIntrinsics4 = "client_4\\Intrinsics_Calib_4.json";
    int camID4 = 4;

    std::string fnameRGB5 = "client_5\\Color_130.jpg";
    std::string fnameMATTE5 = "client_5\\Color_130.matte.png";
    std::string fnameDEPTH5 = "client_5\\Depth_130.tiff";// 
    std::string fnameIntrinsics5 = "client_5\\Intrinsics_Calib_5.json";
    int camID5 = 5;




    std::vector<std::string> pathsRGB;
    std::vector<std::string> pathsMATTE;
    std::vector<std::string> pathsDEPTH;
    std::vector<std::string> pathsINTRINSICS;
    std::vector<int>cameraIDS;

       pathsRGB.push_back(path + fnameRGB0);
  pathsMATTE.push_back(path + fnameMATTE0);
  pathsDEPTH.push_back(path + fnameDEPTH0);
  pathsINTRINSICS.push_back(path + fnameIntrinsics0);
  cameraIDS.push_back(camID0);

    // front side
  pathsRGB.push_back(path + fnameRGB1);
    pathsMATTE.push_back(path + fnameMATTE1);
    pathsDEPTH.push_back(path + fnameDEPTH1);
    pathsINTRINSICS.push_back(path + fnameIntrinsics1);
    cameraIDS.push_back(camID1);


 
    // back angle
    pathsRGB.push_back(path + fnameRGB2);
    pathsMATTE.push_back(path + fnameMATTE2);
    pathsDEPTH.push_back(path + fnameDEPTH2);
    pathsINTRINSICS.push_back(path + fnameIntrinsics2);
    cameraIDS.push_back(camID2);

    ////
    pathsRGB.push_back(path + fnameRGB3);
    pathsMATTE.push_back(path + fnameMATTE3);
    pathsDEPTH.push_back(path + fnameDEPTH3);
    pathsINTRINSICS.push_back(path + fnameIntrinsics3);
    cameraIDS.push_back(camID3);
 
    pathsRGB.push_back(path + fnameRGB4);
    pathsMATTE.push_back(path + fnameMATTE4);
    pathsDEPTH.push_back(path + fnameDEPTH4);
    pathsINTRINSICS.push_back(path + fnameIntrinsics4);
    cameraIDS.push_back(camID4);


        // backside
    pathsRGB.push_back(path + fnameRGB5);
    pathsMATTE.push_back(path + fnameMATTE5);
    pathsDEPTH.push_back(path + fnameDEPTH5);
    pathsINTRINSICS.push_back(path + fnameIntrinsics5);
    cameraIDS.push_back(camID5);

    LoadExtrinsics(fnameExtrinsics);

    /* load in all of the files */
    for (int CAMERA = 0; CAMERA < pathsRGB.size(); CAMERA++)
    {
        int CID = cameraIDS[CAMERA];
      

        std::cout << "CAMERA:" << CAMERA << std::endl;
        cv::Mat imRGB, imMATTE, imDEPTH16, imDEPTH16_transformed, imDEPTH8, imDEPTH32F;
        imRGB = cv::imread(pathsRGB[CAMERA]);
        imMATTE = cv::imread(pathsMATTE[CAMERA]);
        imDEPTH16 = cv::imread(pathsDEPTH[CAMERA],cv::IMREAD_ANYDEPTH ); // 16bit short
        //cv::Mat imDEPTH162 = cv::Mat::zeros(imDEPTH16.rows, imDEPTH16.cols, CV_16UC1);
       // int numChannels = imDEPTH16.channels();
//        if (imDEPTH16.channels() == 3) {
//            for(int j=0;j<imDEPTH16.rows;j++)
//                for (int i = 0; i < imDEPTH16.cols;i++) {
//                    cv::Vec3w pix = imDEPTH16.at<cv::Vec3w>(j, i);
//                    imDEPTH162.at<ushort>(j, i) = pix(0);
//            }
////            cv::extractChannel(imDEPTH16, imDEPTH162, 0);
//
////            cv::cvtColor(imDEPTH16, imDEPTH162, cv::COLOR_BGR5552GRAY);
//        }
//        /* transform depth to RGB size */
        imDEPTH16_transformed = cv::Mat::zeros(imRGB.rows, imRGB.cols, CV_16UC1);
//  // draw the camera extrinsics 
        LoadIntrinsics(pathsINTRINSICS[CAMERA], CID);
        k4a_image_t k4a_pc;
        TransformDepth(imDEPTH16, imDEPTH16_transformed, k4aCalibrations[CID], k4a_pc);
//
        imDEPTH16_transformed.convertTo(imDEPTH8, CV_8UC1, 1.0f / 256.f); // 8bit uchar
//
      //  imDEPTH16_transformed.convertTo(imDEPTH32F, CV_32F);
        //imDEPTH32F.convertTo(imDEPTH8, CV_8UC1);
       // imDEPTH32F.convertTo(imDEPTH8, CV_8UC1, 1.0f / 256.f); // 8bit uchar
    /*    cv::normalize(imDEPTH8, imDEPTH8, 0, 255, cv::NORM_MINMAX);
       cv::imshow("depth", imDEPTH8);
        cv::waitKey();*/
                                                                          
      
       
        Eigen::Matrix4d e = extrinsics[CID];
    //    std::cout << "e:" << e << std::endl;
        Vector3f ax(e(0, 0), e(1, 0), e(2, 0));
        Vector3f ay(e(0, 1), e(1, 1), e(2, 1));
        Vector3f az(e(0, 2), e(1, 2), e(2, 2));
        Vector3f p(e(0, 3), e(1, 3), e(2, 3));
   /*     std::cout << "ax:" << ax << std::endl;
        std::cout << "ay:" << ay << std::endl;
        std::cout << "az:" << az << std::endl;
        std::cout << "p:" << p << std::endl;*/

        auto& axisX = viewer.add_line(p, p+ax,Vector3f(1,0,0));
        auto& axisY = viewer.add_line(p, p + ay, Vector3f(0, 1, 0));
        auto& axisZ = viewer.add_line(p, p + az, Vector3f(0, 0, 1));
        auto& axisZ2 = viewer.add_line(p, p + az*3, Vector3f(0, 0, 0.5));

      //  imDepth = cv::imread(pathsDEPTH[CAMERA]);
       // cv::Mat imMATTE2;
       // cv::Mat rgb2;
       // imRGB.copyTo(rgb2);
        //imMATTE.convertTo(imMATTE2, CV_8UC1);
        //std::cout << "num:" << imMATTE.channels() << std::endl;
        //for (int j = 0; j < 720; j++) {
        //    for (int i = 0; i < 1280; i++) {
        //        cv::Vec3b m = imMATTE2.at<cv::Vec3b>(j, i);
        //        if (m[0] > 0) {

        //            //                rgb2.at<cv::Vec3b>(j, i) = imRGB.at<cv::Vec3b>(j, i);
        //                         //   rgb2.at<uint8_t>(j, i, 1) = imRGB.at<uint8_t>(j, i, 1);
        //                           // rgb2.at<uint8_t>(j, i, 2) = imRGB.at<uint8_t>(j, i, 2);
        //        }
        //        else {
        //            cv::Vec3b& p = rgb2.at<cv::Vec3b>(j, i);
        //            p[0] = 0;
        //            p[1] = 0;
        //            p[2] = 0;
        //        }
        //    }
        //}
       // cv::imshow("rgb", imRGB);
       // cv::imshow("rgb2", rgb2);
        //cv::waitKey();
       // int camera = cameraIDS[CAMERA];
       
        CarveWithSilhouette(theVolume, intrinsics[CID], extrinsics[CID], imRGB, imMATTE, imDEPTH16_transformed, k4a_pc);
       
    }
    AddVolumeToViewer(theVolume);
   // viewer.cull_face = false;
    
  
   // glDisable(GL_CULL_FACE);
    // * Events: key handler
    viewer.on_key = [&](int button, input::Action action, int mods) -> bool {
        if (action != input::Action::release) {
            if (button == 'd') {
               
            }
            else if (button == 'e') {
                

            }
        }
        return true;  // Don't prevent default
    };
    // * Events: loop callback
    viewer.on_loop = [&]() -> bool {
        return false;  // True to update all meshes and camera
    };
    
    viewer.on_gui = [&]() -> bool {
        ImGui::SetNextWindowSize(ImVec2(200, 100));
        ImGui::Begin("Hello");
        ImGui::Text("Hello world");
        ImGui::Button("Panic button");
        ImGui::End();
        return false;  // True to update all meshes and camera
    };

    viewer.show();
}
