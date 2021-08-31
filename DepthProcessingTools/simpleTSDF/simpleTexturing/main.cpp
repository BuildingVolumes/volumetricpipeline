#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>

#include "meshview/meshview.hpp"
#include "meshview/meshview_imgui.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "TSDFVolume.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>
#include <k4a/k4a.h>

#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>

#define LARGE_F 1000000.f
struct MyTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3f Point;
    VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color | OpenMesh::Attributes::TexCoord2D);
    HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
    FaceAttributes(OpenMesh::Attributes::Normal);
    FaceTraits
    {
    public:
        float energy;
        int camera;
        FaceT() : energy(LARGE_F), camera(-1) {}
       /* const float energy() const { return energy; }
        const int camera() const { return camera; }
        void set_energy(const float& f) { energy = f; }
        void set_camera(const int& ind) { camera = ind; }*/
    };
  
   
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;

class LiveScan3d_Client {
public:
    LiveScan3d_Client(std::string _take_dir, int clientID) : take_dir(_take_dir), id(clientID){
        k4a_pc = nullptr;
        transform = 0;
        // get names of all things inside this directory 
        client_dir = take_dir + "\\client_" + std::to_string(id);
        intrinsics_fname = client_dir + "\\Intrinsics_Calib_" + std::to_string(id) + ".json";
        timestamps_fname = client_dir + "\\Timestamps_Client" + std::to_string(id) + ".txt";
        prefix_Color = client_dir + "\\Color_";
        prefix_Depth = client_dir + "\\Depth_";
    }

    void TransformDepth(cv::Mat& old_depth, cv::Mat& new_depth, k4a_calibration_t& calibration, k4a_image_t& k4a_pointcloud) {
        k4a_image_t k4a_transformed_depth = nullptr;
        k4a_image_t k4a_depth = nullptr;

        int oldStride = old_depth.step[0];
        if (K4A_RESULT_SUCCEEDED !=
            k4a_image_create_from_buffer(k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, old_depth.cols, old_depth.rows, oldStride,
            old_depth.data, old_depth.step[0] * old_depth.rows, nullptr, nullptr, &k4a_depth))
        {
            std::cout << "Transform Depth error: failed to create k4a image" << std::endl;
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
            new_depth.data, new_depth.step[0] * new_depth.rows, nullptr, nullptr, &k4a_transformed_depth))
        {
            std::cout << "error k4a_image_create_from_buffer" << std::endl;
        }

        if (transform == 0) {
            transform = k4a_transformation_create(&calibration);
        }

        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transform, k4a_depth, k4a_transformed_depth))
        {
            std::cout << "error transforming depth to rgb" << std::endl;
        }
        if (k4a_pointcloud == NULL)
        {
            k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, new_depth.cols, new_depth.rows, new_depth.cols * 3 * (int)sizeof(int16_t), &k4a_pointcloud);
        }
        k4a_transformation_depth_image_to_point_cloud(transform, k4a_transformed_depth, K4A_CALIBRATION_TYPE_COLOR, k4a_pointcloud);


        // release memory
        k4a_image_release(k4a_depth);
        k4a_image_release(k4a_transformed_depth);
    }

    void enumerate() {
        size_t found=0;
        for (const auto& entry : std::filesystem::directory_iterator(client_dir)) {
            std::string p = entry.path().generic_string();
            // get frame num
           // std::cout << "p:" << p << std::endl;
            std::string pfix_color = "Color_";
            std::string pfix_depth = "Depth_";

            if ((found = p.find(pfix_color)) != std::string::npos) {
                size_t found2 = p.find(".", found + 1);
                std::string frame_s = p.substr(found+pfix_color.length(), found2 - (found+ pfix_color.length()));
                int frame = std::atoi(frame_s.c_str());
                if (p.find("matte") != std::string::npos) {
                    matte_names[frame] = p;
                  //  std::cout << "matte:" << frame << ":" << p << std::endl;
                }
                else {
                    // not a matte, just a color image
                    color_names[frame] = p;
                    //std::cout << "color:" << frame << ":" << p << std::endl;
                }
            }
            else if ((found = p.find(pfix_depth)) != std::string::npos) {
                size_t found2 = p.find(".", found + 1);
                int frame = std::atoi(p.substr(found+pfix_depth.length(), found2 - (found + pfix_depth.length())).c_str());
                if (entry.path().extension() == ".png") {
                    depth_names[frame] = p;
                    //std::cout << "depth:" << frame << ":"<<p<<std::endl;
                }
            }
        }
    }
    void LoadIntrinsics() {
        
        std::ifstream intrinsic_file = std::ifstream(intrinsics_fname);
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

        intrinsics = Eigen::Matrix3d::Identity();
        intrinsics(0, 0) = params.param.fx;
        intrinsics(1, 1) = params.param.fy;
        intrinsics(0, 2) = params.param.cx;
        intrinsics(1, 2) = params.param.cy;
        std::cout << intrinsics << std::endl;
        //intrinsics[cameraNum] = intrinsic_mat;
        //k4aCalibrations[cameraNum] = calibration;
    }
    void SetExtrinsics(Eigen::Matrix4d &m) {
        extrinsics = m;

        auto ExInv = extrinsics;
        auto R = extrinsics.block<3, 3>(0, 0);
        auto T = extrinsics.block<3, 1>(0, 3);
        auto Rt = R.transpose();
        auto T2 = -Rt * T;
        ExInv.block<3, 3>(0, 0) = Rt;
        ExInv.block<3, 1>(0, 3) = T2;


        extrinsicsInv = ExInv;// extrinsics.inverse();
        std::cout << "e:"<<extrinsics << std::endl;
        std::cout << "eInv:"<<extrinsicsInv << std::endl;
    }
    Eigen::Matrix3d& GetIntrinsics() {
        return intrinsics;
    }
    Eigen::Matrix4d& GetExtrinsicsInv() {
        return extrinsicsInv;
    }
    cv::Mat& GetRGB() {
        return imRGB;
    }
    cv::Mat& GetMatte() {
        return imMATTE;
    }
    cv::Mat& GetDepthRaw() {
        return imDEPTH16;
    }
    cv::Mat& GetDepth16Transformed() {
        return imDEPTH16_transformed;
    }


    void LoadFrame(int f) {

       imRGB = cv::imread(color_names[f]);
       imMATTE = cv::imread(matte_names[f]);
       imDEPTH16 = cv::imread(depth_names[f], cv::IMREAD_ANYDEPTH); // 16bit short

       /* transform depth to RGB size */
       imDEPTH16_transformed = cv::Mat::zeros(imRGB.rows, imRGB.cols, CV_16UC1);

       TransformDepth(imDEPTH16, imDEPTH16_transformed, calibration, k4a_pc);

       // now all images are loaded for this frame
    }
    ~LiveScan3d_Client() {}
    std::string take_dir;
    std::string client_dir;
    std::string intrinsics_fname;
    std::string timestamps_fname;
    std::string prefix_Color;
    std::string prefix_Depth;
    std::map<int, std::string> color_names;
    std::map<int, std::string> depth_names;
    std::map<int, std::string> matte_names;
    Eigen::Matrix4d extrinsics;
    Eigen::Matrix4d extrinsicsInv;

    Eigen::Matrix3d intrinsics;
    k4a_calibration_t calibration;

    int id;

    // each camera needs these to be loaded
    cv::Mat imRGB, imMATTE, imDEPTH16, imDEPTH16_transformed;
    k4a_image_t k4a_pc;
    k4a_transformation_t transform;
};

class LiveScan3d_Take {
public:
    LiveScan3d_Take(std::string _base_dir, std::string _extrinsics_name, int _numClients): base_dir(_base_dir) {
        extrinsics_filename = base_dir + "\\" + _extrinsics_name;
        numClients = _numClients;

        for (int i = 0; i < numClients; i++) {
            LiveScan3d_Client theClient(base_dir,i);
            theClient.enumerate();
            clients.push_back(theClient);
        }
        LoadIntrinsics();
        LoadExtrinsics();
    }
    Eigen::Matrix3d &GetIntrinsics(int clientInd) {
        return clients[clientInd].GetIntrinsics();
    }
    Eigen::Matrix4d &GetExtrinsicsInv(int clientInd) {
        return clients[clientInd].GetExtrinsicsInv();
    }
    cv::Mat& GetRGB(int clientInd) {
        return clients[clientInd].GetRGB();
    }

    cv::Mat& GetMatte(int clientInd) {
        return clients[clientInd].GetMatte();
    }
    cv::Mat& GetDepthRaw(int clientInd) {
        return clients[clientInd].GetDepthRaw();
    }
    cv::Mat& GetDepth16Transformed(int clientInd) {
        return clients[clientInd].GetDepth16Transformed();
    }
    void LoadIntrinsics() {
        // go through each client and load their intrinsics
        for (int i = 0; i < numClients; i++) {
            clients[i].LoadIntrinsics();
        }
    }
    void LoadExtrinsics() {
        std::fstream extrinsics_filestream;
        extrinsics_filestream.open(extrinsics_filename);
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

            clients[theCam].SetExtrinsics(theMat);
            //extrinsics[theCam] = theMat;// .inverse();
            count++;
        }
    }
    void LoadFrame(int f)
    {
        // load images for this frame into memory and do any/all pre-processing for each client
        for (int i = 0; i < numClients; i++) {
            clients[i].LoadFrame(f);
       }
    }

    ~LiveScan3d_Take() {}
    std::string base_dir;    
    std::string extrinsics_filename;
    std::vector<LiveScan3d_Client> clients;

    
    int numClients;
};



Eigen::Vector3d ProjectPoint(Eigen::Vector3d& p, Eigen::Matrix3d& intrin, Eigen::Matrix4d& exInv, Eigen::Vector4d& cc) {
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

int main(int argc, char** argv) {
//    std::vector<TRIANGLE> mesh;
    MyMesh mesh;
    if(!OpenMesh::IO::read_mesh(mesh, "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_im2-remNonManFaces-cut0.obj"))
    {
        std::cerr << "read error" << std::endl;
        exit(1);
    }

    std::cout << "loaded: faces: " << mesh.n_faces() << std::endl;
    std::cout << "loaded: verts: " << mesh.n_vertices() << std::endl;
    // Initialize subdivider
    //OpenMesh::Subdivider::Adaptive::CompositeT<MyMesh> adapt(mesh);
    //
    //OpenMesh::Subdivider::Uniform::CatmullClarkT<MyMesh> catmull;
    ////// Execute 1 subdivision steps
    //catmull.attach(mesh);
    //catmull(3);
    //catmull.detach();
    // Initialize smoother with input mesh
   // OpenMesh::Smoother::JacobiLaplaceSmootherT<MyMesh> smoother(mesh);
   // smoother.initialize(OpenMesh::Smoother::JacobiLaplaceSmootherT<MyMesh>::Component::Tangential_and_Normal,   //Smooth direction
   //                     OpenMesh::Smoother::JacobiLaplaceSmootherT<MyMesh>::Continuity::C0);                      //Continuity

   //// Execute 3 smooth steps
   //     smoother.smooth(3);

    // now load in Livescan Take info
    LiveScan3d_Take theTake("C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0","outputExtrinsics.log",6);
    
    theTake.LoadFrame(160);

    // now we have the camera info and the mesh info
    // let's do some texturing

    // stupid texturing first
    // greedy version
    //  do one camera and project mesh onto it
    //   compute energy of this camera and this face
       //  if no texture stored, then simply store energy and u,v    
    //     if energy already exists, check for minimum and keep min
    //  
    Eigen::Matrix3d intrin;
    Eigen::Matrix4d exInv; 
    Eigen::Vector4d cc;
    int CLIENT = 0;
    int CAMERA_TO_SET = -1;


    for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        //if (mesh.data(*f_it).camera == -1) 
        {
            auto vr = f_it->vertices();
            mesh.data(*f_it).energy = LARGE_F;
            mesh.data(*f_it).camera = -1;
            for (auto vit = vr.begin(); vit != vr.end(); ++vit) {
                auto uv = mesh.texcoord2D(*vit);
                uv[0] = uv[1] = 0;
                mesh.set_texcoord2D(*vit, uv);
            }
        }
    }




    // foreach face in the mesh
    for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) 
    {
        auto vr = f_it->vertices();
        MyMesh::Point p;
        Eigen::Vector3d v;
        p[0] = p[1] = p[2] = 0;
        float energy = mesh.data(*f_it).energy;
        CAMERA_TO_SET = mesh.data(*f_it).camera;
        std::map<int, float> costPerCamera; // for this face costPerCamera[i] is the computed cost for the i'th camera
        /* project this face onto each camera and compute the cost */
        std::map<int, std::vector<MyMesh::TexCoord2D>> texCoordsPerCamera;
        for (int CAMERA = 0; CAMERA < 6; CAMERA++)
        {
            intrin = theTake.GetIntrinsics(CAMERA);
            exInv = theTake.GetExtrinsicsInv(CAMERA);

            cv::Mat imRGB = theTake.GetRGB(CAMERA);
            cv::Mat imMATTE = theTake.GetMatte(CAMERA);
            cv::Mat imDepth16Transformed = theTake.GetDepth16Transformed(CAMERA);
            std::vector<MyMesh::TexCoord2D> texCoords; // this will contain the tex coords for all verts in this face for this camera
            costPerCamera[CAMERA] = 0; // face cost, sum of cost for each vert 
            /* project all vertices in this face onto this camera */
            for (auto vit = vr.begin(); vit != vr.end(); ++vit) {
                // get the vertex value
                p = mesh.point(*vit);
                v[0] = p[0]; v[1] = p[1]; v[2] = p[2];

                Eigen::Vector3d proj = ProjectPoint(v, intrin, exInv, cc);
                int u, v;
                u = (int)proj(0); v = (int)proj(1);
                float uvz = proj(2); // this is the depth of the voxel from the camera
                if (u > 0 && u < imRGB.cols && v > 0 && v < imRGB.rows)
                {
                    // it projected into the image so that's good
                    cv::Vec3b m = imMATTE.at<cv::Vec3b>(v, u);  // get matte pixel value
                    cv::Vec3b col = imRGB.at<cv::Vec3b>(v, u); // get colour
                    ushort depth = imDepth16Transformed.at<ushort>(v, u); // get depth image value  
                    float df = (float)(depth) / 1000.f;
                    float diff = df - uvz;
                    MyMesh::TexCoord2D uv;
                    uv[0] = (float)u / (float)imRGB.cols;
                    uv[1] =  (float)v / (float)imRGB.rows;
                    texCoords.push_back(uv);

                    costPerCamera[CAMERA] += fabs(diff); // stupid silly cost at the moment
                }
                else {
                    MyMesh::TexCoord2D uv;
                    uv[0] = 0;//(float)u / (float)imRGB.cols;
                    uv[1] = 0;//(float)v / (float)imRGB.rows;
                    texCoords.push_back(uv);
                    // didn't project into the image boundaries
                    costPerCamera[CAMERA] = LARGE_F; // huge cost if it doesn't project
                }
               // std::cout << "COST:" << costPerCamera[CAMERA] << std::endl;
                texCoordsPerCamera[CAMERA] = texCoords;
            }// end per vert loop
        }// end for camera
        /* find minimum costs */
        float minCost = LARGE_F;
        float minCamera = 100;
        for (int i = 0; i < costPerCamera.size(); i++) {
            if (costPerCamera[i] < minCost) {
                minCost = costPerCamera[i];
                minCamera = i;
            }
        }
        
        //std::cout << "MINCAM:" << minCamera << std::endl;
        /* minCamera is the camera with the minimum cost for this face */
        /* store the texture computed texture coordinates here */
        //vr = f_it->vertices();
        auto tc = texCoordsPerCamera[minCamera];
        int vind = 0;
        float offset = (1.0f/6.f)*minCamera;
        float scale = 1.0f/6.f;
        for (auto vit = vr.begin(); vit != vr.end(); ++vit) {
            auto uv = mesh.texcoord2D(*vit);
            uv[0] = uv[1] = 0;

            if (minCost >= LARGE_F) {
                uv[0] = uv[1] = 0;
            }
            else {
                uv[0] = (tc[vind])[0];
                uv[1] = 1.0f - ((tc[vind])[1] * scale + offset);
            }
            mesh.set_texcoord2D(*vit, uv);
            vind++;
        }
    }// end for face

    /*for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
        if (mesh.data(*f_it).camera == -1) {
            auto vr = f_it->vertices();
            for (auto vit = vr.begin(); vit != vr.end(); ++vit) {
                auto uv = mesh.texcoord2D(*vit);
                uv[0] = uv[1] = 0;
                mesh.set_texcoord2D(*vit, uv);
            }
        }
    }*/

    OpenMesh::IO::write_mesh(mesh, "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_proc.obj",
                             OpenMesh::IO::Options::Flag::VertexTexCoord);

	return 0;
}