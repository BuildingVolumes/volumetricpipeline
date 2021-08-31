#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>

#include "meshview/meshview.hpp"
#include "meshview/meshview_imgui.hpp"
#include "TSDFVolume.h"
#include "LiveScan3D_utils.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include "main.h"

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

/* project a point onto our camera using intrinsics and extrinsics */
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

/* init custom data required in our mesh */
void initializeFaceData(MyMesh &mesh) {
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
}
/* compute the cost of the projection of this face on this camera */
float computeFaceCost(LiveScan3d_Take &theTake, 
                      int CAMERA, 
                      OpenMesh::PolyConnectivity::ConstFaceVertexRange &vr, 
                      MyMesh &mesh, 
                      std::map<int, std::vector<MyMesh::TexCoord2D>> &texCoordsPerCamera)
{
    MyMesh::Point p;
    Eigen::Vector3d v;
    p[0] = p[1] = p[2] = 0;
    float cost = 0;
    Eigen::Matrix3d intrin;
    Eigen::Matrix4d exInv;
    Eigen::Vector4d cc;
    intrin = theTake.GetIntrinsics(CAMERA);
    exInv = theTake.GetExtrinsicsInv(CAMERA);
    std::vector<MyMesh::TexCoord2D> texCoords;
    cv::Mat imRGB = theTake.GetRGB(CAMERA);
    cv::Mat imMATTE = theTake.GetMatte(CAMERA);
    cv::Mat imDepth16Transformed = theTake.GetDepth16Transformed(CAMERA);
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
            uv[1] = (float)v / (float)imRGB.rows;
            texCoords.push_back(uv);

            cost += fabs(diff); // stupid silly cost at the moment
        }
        else {
            MyMesh::TexCoord2D uv;
            uv[0] = 0;//(float)u / (float)imRGB.cols;
            uv[1] = 0;//(float)v / (float)imRGB.rows;
            texCoords.push_back(uv);
            // didn't project into the image boundaries
            cost = LARGE_F; // huge cost if it doesn't project
        }
    }
    texCoordsPerCamera[CAMERA] = texCoords;
    return cost;
}

/* set the texture coordinates properly for this face*/
void SetTexCoords(MyMesh &mesh, 
                  float minCost, float minCamera,
                  OpenMesh::PolyConnectivity::ConstFaceVertexRange& vr,  
                  std::map<int, std::vector<MyMesh::TexCoord2D>> &texCoordsPerCamera){
    auto tc = texCoordsPerCamera[minCamera];
    int vind = 0;
    float offset = (1.0f / 6.f) * minCamera;
    float scale = 1.0f / 6.f;
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
}

/* look in the map for the minimum cost and camera for this face */
float GetMinCost(std::map<int, float> &costPerCamera, int &minCamera) {
    float minCost = LARGE_F;
    for (int i = 0; i < costPerCamera.size(); i++) {
        if (costPerCamera[i] < minCost) {
            minCost = costPerCamera[i];
            minCamera = i;
        }
    }
    return minCost;
}

/* THE MAIN FUNCTION */
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

    // now load in Livescan Take info
    int NUM_CAMERAS = 6;
    LiveScan3d_Take theTake("C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0","outputExtrinsics.log",NUM_CAMERAS);
    
    theTake.LoadFrame(160);

    /* initialize face data */
    initializeFaceData(mesh);

    // foreach face in the mesh
    for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) 
    {
        OpenMesh::PolyConnectivity::ConstFaceVertexRange vr = f_it->vertices();
        std::map<int, float> costPerCamera; 
 
        /* project this face onto each camera and compute the cost */
        std::map<int, std::vector<MyMesh::TexCoord2D>> texCoordsPerCamera;
        for (int CAMERA = 0; CAMERA < NUM_CAMERAS; CAMERA++)
        {
            costPerCamera[CAMERA] = computeFaceCost(theTake, CAMERA, vr, mesh, texCoordsPerCamera);
        }

        /* find minimum costs */
        float minCost = LARGE_F;
        int minCamera = 100;
        minCost = GetMinCost(costPerCamera, minCamera);
        
        /* minCamera is the camera with the minimum cost for this face */
        /* store the texture computed texture coordinates here */
        SetTexCoords(mesh, minCost, minCamera, vr, texCoordsPerCamera);
    }// end for face

    // save
    OpenMesh::IO::write_mesh(mesh, 
                             "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_proc.obj",
                             OpenMesh::IO::Options::Flag::VertexTexCoord);

	return 0;
}