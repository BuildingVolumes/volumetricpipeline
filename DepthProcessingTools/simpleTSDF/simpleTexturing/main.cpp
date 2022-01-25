#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>

#include <uvpCore.hpp>

#include "meshview/meshview.hpp"
#include "meshview/meshview_imgui.hpp"
#include "TSDFVolume.h"
#include "LiveScan3D_utils.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "main.h"

// Texture packer stuff
//#include "TexturePacker.h"
#include "MeshUtils.h"

//// ceres solver stuff
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/cubic_interpolation.h"




/* project a point onto our camera using intrinsics and extrinsics */
Eigen::Vector3d ProjectPoint(Eigen::Vector3d& p, const Eigen::Matrix3d& intrin, const Eigen::Matrix4d& exInv, Eigen::Vector4d& cc) {
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
                mesh.data(*vit).fh = (*f_it);
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

int ClusterVertices(MyMesh& mesh, float radius) {
    // ok, so here for efficiency we should use a spatial hash table to find close verts
    OpenMesh::Vec3f binSize;
    binSize[0] = 1;
    binSize[1] = 1;
    binSize[2] = 1;
   

    SpatialHashTable SHT(mesh, radius, 64);
   
    // for each vertex in the mesh
    //  add the vertex into the spatial hash bucket
    int nV = mesh.n_vertices();
    for (int i = 0; i < nV; ++i)
    {
        OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(i);
        SHT.insert(vh);
    }
    // once we have added all vertices into the spatial hash table
    // we have a number of buckets contiaining a list of vertex handle
    //OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(nV/2);
    //OpenMesh::Vec3f p = mesh.point(vh);
    //int3 bin = SHT.GetBinKey(p);
    //std::vector<int3> nn = SHT.GetValidNeighbours(bin, 3);
    //std::cout << "nn.size()=" << nn.size() << std::endl;

    // for each vertex in the mesh
    //   determine which bin in the hash we should look at
    for (int i = 0; i < nV; ++i)
    {
        OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(i);
        OpenMesh::Vec3f p = mesh.point(vh);
 
        std::vector<OpenMesh::VertexHandle> mergeCandidates = SHT.GetClosestPointsWithinRadius(vh, radius);
        if (mergeCandidates.size() > 0)
        {
            // merge the verts with p
            std::cout << "Vertex( "<<i<<") - Num Merge Candidates: " << mergeCandidates.size() << std::endl;
            // mergeCandidates contains only vertexhandles that are seen in the same camera also
            // so we just have to go through and connect the candidate's faces to point to p properly....
            for (OpenMesh::VertexHandle toMerge : mergeCandidates) {
                // maybe look at insert_edge() and delete_edge()....
                // something to do with the halfedges
                // 

                // outgoing halfedge from vh
              //OpenMesh::HalfedgeHandle heh =  mesh.halfedge_handle(vh);

               // set the outgoing halfedge of a given vertex
               //mesh.set_halfedge_handle(vh, heh);

               // next and previous and opposite half edge handles
               //OpenMesh::HalfedgeHandle neh = mesh.next_halfedge_handle(heh);
               //OpenMesh::HalfedgeHandle peh = mesh.prev_halfedge_handle(heh);
               //OpenMesh::HalfedgeHandle oeh = mesh.opposite_halfedge_handle(heh);
                
               // set the next halfedge handle
               //mesh.set_next_halfedge_handle(heh, neh);
            
            }
            // and then remove the candidates from the mesh
        }
    }
    
    return 0;
}

void RemoveDuplicateVertices(MyMesh& mesh, bool flag) {

}

int MergeCloseVertices(MyMesh &mesh, double radius) {
    int mergedCount = 0;
    mergedCount = ClusterVertices(mesh, radius);
    RemoveDuplicateVertices(mesh, true);
    return mergedCount;
}

void testMesh()
{
    MyMesh mesh;
    // generate vertices
    MyMesh::VertexHandle vhandle[8];
    vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1, 1));
    vhandle[1] = mesh.add_vertex(MyMesh::Point(1, -1, 1));
    vhandle[2] = mesh.add_vertex(MyMesh::Point(1, 1, 1));

    vhandle[3] = mesh.add_vertex(MyMesh::Point(0.8, 1, 1));
    vhandle[4] = mesh.add_vertex(MyMesh::Point(-0.8, 1, 1));
    vhandle[5] = mesh.add_vertex(MyMesh::Point(-0.8, -1, 1));


    //vhandle[3] = mesh.add_vertex(MyMesh::Point(-1, 1, 1));
    //vhandle[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
    //vhandle[5] = mesh.add_vertex(MyMesh::Point(1, -1, -1));
    //vhandle[6] = mesh.add_vertex(MyMesh::Point(1, 1, -1));
    //vhandle[7] = mesh.add_vertex(MyMesh::Point(-1, 1, -1));
    // generate (quadrilateral) faces
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
   // face_vhandles.push_back(vhandle[3]);
    auto fh2 = mesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[5]);
//    face_vhandles.push_back(vhandle[4]);
   auto fh3 =  mesh.add_face(face_vhandles);



   // now process this mesh somehow
   OpenMesh::HalfedgeHandle h = mesh.halfedge_handle(vhandle[4]);
   OpenMesh::HalfedgeHandle hn = mesh.next_halfedge_handle(h);
   OpenMesh::HalfedgeHandle hp = mesh.prev_halfedge_handle(h);
   
   OpenMesh::HalfedgeHandle o = mesh.opposite_halfedge_handle(h);
   OpenMesh::HalfedgeHandle on = mesh.next_halfedge_handle(o);
   OpenMesh::HalfedgeHandle op = mesh.prev_halfedge_handle(o);

   OpenMesh::FaceHandle fh = mesh.face_handle(h);
   OpenMesh::FaceHandle fo = mesh.face_handle(o);

   OpenMesh::VertexHandle vh = mesh.to_vertex_handle(h);
   OpenMesh::VertexHandle vo = mesh.to_vertex_handle(o);
   OpenMesh::Vec3f ph = mesh.point(vh);
   OpenMesh::Vec3f po = mesh.point(vo);

   OpenMesh::VertexHandle vh2 = mesh.from_vertex_handle(h);
   OpenMesh::VertexHandle vo2 = mesh.from_vertex_handle(o);
   OpenMesh::Vec3f ph2 = mesh.point(vh2);
   OpenMesh::Vec3f po2 = mesh.point(vo2);

   std::cout << "ph to:" << ph[0] << "," << ph[1] << "," << ph[2] << std::endl;
   std::cout << "po to:" << po[0] << "," << po[1] << "," << po[2] << std::endl;

   std::cout << "ph2 from:" << ph2[0] << "," << ph2[1] << "," << ph2[2] << std::endl;
   std::cout << "po2 from:" << po2[0] << "," << po2[1] << "," << po2[2] << std::endl;




    mesh.garbage_collection();

    //
//    auto fh5 = mesh.face_handle(heh5);


    //OpenMesh::HalfedgeHandle heh3 = mesh.halfedge_handle(vhandle[3]);
    //OpenMesh::HalfedgeHandle neh3 = mesh.next_halfedge_handle(heh3); // does this go to 4 ?
    //mesh.set_next_halfedge_handle(heh2, neh3);
    
    // how do I merge(2,3) and merge(0,5)
   // OpenMesh::HalfedgeHandle heh0 = mesh.halfedge_handle(vhandle[0]);
   
  //  OpenMesh::HalfedgeHandle neh5 = mesh.next_halfedge_handle(heh5); // does this go to 4 ?
  //  mesh.set_next_halfedge_handle(heh0, neh5);
    //mesh.delete_vertex(vhandle[3]);
    //mesh.delete_vertex(vhandle[5]);
    // take a look at this which I think is how to do it properly..... or at least hints
// https://graphics.rwth-aachen.de:9000/OpenFlipper-Free/Plugin-MeshRepair/-/blob/master/NonManifoldVertexFixingT_impl.hh

    // delete vertex 3, add edge between 2,4
    // delete vertex 5, add edge between 0,4


    /*face_vhandles.clear();
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[5]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[6]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[7]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[4]);
    mesh.add_face(face_vhandles);*/

    OpenMesh::IO::write_mesh(mesh, "output.obj");
}

void ComputeUVs(MyMesh &mesh, LiveScan3d_Take &theTake) {
    // now load in Livescan Take info
    int NUM_CAMERAS = theTake.numClients;

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

    /* at this point I am assuming that I have a triangle-soup mesh
    - each face has 3 verts, each vert has a UV and an assigned camera
    */
    /* next step is to fix the mesh and weld the face verts
       going to base this implementation on the VCGLib (meshlab) implementation of MergeCloseVertex()

     */
    //float radius = 0.0001;
    /* int numMerged = MergeCloseVertices(mesh, radius);

     return 0;*/
}
void PackUVs(MyMesh& mesh, LiveScan3d_Take& theTake) {
    //MyMesh mesh;
    //if (!OpenMesh::IO::read_mesh(mesh, "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_proc-mergedvertswithuvs.obj"))
    //{
    //    std::cerr << "read error" << std::endl;
    //    exit(1);
    //}

    //std::cout << "loaded: faces: " << mesh.n_faces() << std::endl;
    //std::cout << "loaded: verts: " << mesh.n_vertices() << std::endl;
    //mesh.triangulate();
    // now load in Livescan Take info
    int NUM_CAMERAS = theTake.numClients;
  /*  LiveScan3d_Take theTake("C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0", "outputExtrinsics.log", NUM_CAMERAS);

    theTake.LoadFrame(160);*/

     /* do Texture packing thing here */
  //  uvpcore::UvpOperationInputT inputUVP;
   // TexturePacker thePacker;
    // set up the packer info
    //auto better_texture = std::make_shared<open3d::geometry::Image>();
    //better_texture->width_ = 1920; // color_images[0].width_;
    //better_texture->height_ = 1080; // color_images[0].height_;
    //better_texture->bytes_per_channel_ = color_images[0].bytes_per_channel_;
    //better_texture->num_of_channels_ = color_images[0].num_of_channels_;
    //better_texture->data_.resize(better_texture->width_ * better_texture->height_ * better_texture->num_of_channels_ * better_texture->bytes_per_channel_);

    // run the uv pack
    //ErrorLogger::EXECUTE("Perform UV packing", &tu, &TextureUnpacker::PerformTextureUnpack, &color_images, mesh, &(*better_texture), false);




    std::vector<cv::Mat> color_array;
    cv::Mat outputImage;
    for (int i = 0; i < theTake.numClients; i++) {
        color_array.push_back(theTake.GetRGB(i));
    }
    int width, height;
    width = 4096;
    height = 4096;
    outputImage = cv::Mat::zeros(height, width, CV_8UC3);
   // thePacker.PerformTexturePack(mesh, color_array, outputImage, true);

     // get the results
    // cv::imwrite("outputtex.png", outputImage);

    
}

void SaveBundlerFormat(LiveScan3d_Take &theTake) {
    std::ofstream file = std::ofstream("bundler.out");
    file << theTake.numClients << " 0\n";
    for (int i = 0; i < theTake.numClients;i++) {
        auto intrin = theTake.GetIntrinsics(i);
        auto extrin = theTake.GetExtrinsics(i);
        auto kappa = theTake.GetKappa(i);
        float focal, k1, k2;
        float fx, fy;
        fx = intrin(0, 0);
        fy = intrin(1, 1);
        focal = fy;// (fx + fy) * 0.5;
        k1 = kappa[0];
        k2 = kappa[1];
        auto R = extrin.block<3, 3>(0, 0);
        auto t = extrin.block<3, 1>(0, 3);
        std::cout << "extrin:" << extrin << std::endl;
        std::cout << "R:" << R << std::endl;
        std::cout << "t:" << t << std::endl;

        file << focal << " " << k1 << " " << k2 << "\n";
        file << R <<"\n";
        file << t[0] << " " << t[1] << " " << t[2] << "\n";
    }

    file.close();

}


void SaveMLPFormat(LiveScan3d_Take& theTake, std::string fname, std::string meshname, int FRAMENUM) {
    std::ofstream file = std::ofstream(fname, std::ios_base::binary);
    file << "<!DOCTYPE MeshLabDocument>\n";
    file << "<MeshLabProject>\n";
    file << "<MeshGroup>\n";
    file << "<MLMesh filename=\""<<meshname<<"\" visible=\"1\" label=\""<<meshname<<"\">\n";
    file << "<MLMatrix44>\n";
    file << "1 0 0 0 \n0 1 0 0 \n0 0 1 0 \n0 0 0 1 \n";
    file << "</MLMatrix44>\n";
  //  file << "<RenderingOption pointColor=\"131 149 69 255\" wireColor=\"64 64 64 255\" boxColor=\"234 234 234 255\" solidColor=\"192 192 192 255\" wireWidth=\"1\" pointSize=\"3\">100001000000000000000100000001010101000010100000000100111011100000001001 </RenderingOption>\n";
    file << "</MLMesh>\n";
    file << "</MeshGroup>\n";
    file << "<RasterGroup>\n";
    for (int i = 0; i < theTake.numClients; i++) 
    {
        // get data from the take    
        auto intrin = theTake.GetIntrinsics(i);
        auto extrin = theTake.GetExtrinsics(i);
        auto extrinInv = theTake.GetExtrinsicsInv(i);
        auto kappa = theTake.GetKappa(i);
 
        // set the intrinsics for Meshlab
        float k1, k2;
        float fx, fy, cx, cy, focal;
        float pxSx, pxSy, focalMm;
        float width, height;
        width = 1280; height = 720;

        fx = intrin(0, 0);
        fy = intrin(1, 1);
        cx = width / 2;// intrin(0, 2);
        cy = height / 2;// intrin(1, 2);
        focal = fy;// (fx + fy) * 0.5;
    
      
        pxSx = 1; pxSy = 1;
        focalMm = focal;
        k1 = kappa[0] / focal;
        k2 = kappa[1] / focal;

        auto Rt = extrin.block<3, 3>(0, 0).transpose();
        auto t = extrin.block<3, 1>(0, 3);
        std::wcout << "CLIENT:" << i << std::endl;
        std::cout << "extrin:" << extrin << std::endl;
        std::cout << "Rt:" << Rt << std::endl;
        std::cout << "t:" << t << std::endl;

        // meshlab's y and z are flipped, so here I flip the 2,3 rows
        Eigen::Matrix3d S;
        S = S.Identity();
        S(1, 1) = -1;
        S(2, 2) = -1;
        

        auto Rf = S * Rt;
        auto tf = -t;


        // output to file 
        file << "<MLRaster label=\"" << i << "_Color_"<<FRAMENUM<<".jpg\">\n";
        file << "<VCGCamera ";
        file << " CenterPx=\"" << cx << " " << cy << "\" ";
        file << " FocalMm=\"" << focalMm << "\" ";
        file << " TranslationVector=\"" << tf[0] << " " << tf[1] << " " << tf[2] << " 1\" ";
        file << " PixelSizeMm=\"" << pxSx << " " << pxSy << "\"";
        file << " ViewportPx=\""<<width<<" "<<height<<"\"";
        file << " LensDistortion=\"" << k1 << " " << k2 << "\"";
        file << " RotationMatrix=\""
            << Rf(0, 0) << " "
            << Rf(0, 1) << " "
            << Rf(0, 2) << " "
            << "0 "
            << Rf(1, 0) << " "
            << Rf(1, 1) << " "
            << Rf(1, 2) << " "
            << "0 "
            << Rf(2, 0) << " "
            << Rf(2, 1) << " "
            << Rf(2, 2) << " "
            << "0 "
            << "0 0 0 1 \"";
        file << "CameraType=\"0\" />\n";
        file << "<Plane fileName=\"../client_"<<i<<"/Color_"<<FRAMENUM<<".jpg\" semantic=\"1\"/>";
        file << "\n</MLRaster>\n";
    }
    file << "</RasterGroup>\n";
    file << "</MeshLabProject>\n";
    file.close();

}

using Grid = ceres::Grid2D<double>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;

// NOTE: ArrayXXD defaults to a Column-Major ordering
//        Eigen::ArrayXXd img_data = Eigen::ArrayXXd(rows,cols);
// so lets use this alias to force it to be row-major for consistency
using EigArray = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation,  
struct ReprojectionError {
    ReprojectionError(const Interpolator &interpolator, cv::Mat& theSDFimage,
                      MyMesh& theMesh,
                      const Eigen::Matrix3d& intrin,
                      const Eigen::Matrix4d& extrin,
                      int cameraID) : theInterpolator(interpolator), imSDF(theSDFimage), intrin(intrin), extrin(extrin), mesh(theMesh), cameraID(cameraID)
    {
       
    }
    ReprojectionError(const Interpolator& interpolator, 
                      const OpenMesh::Vec3f vert,
                      const Eigen::Matrix3d& intrin,
                      const Eigen::Matrix4d& extrin,
                      int cameraID) : theInterpolator(interpolator), vertex(vert), intrin(intrin), extrin(extrin), cameraID(cameraID)
    {

    }

    /* this is the cost function for the minimizer */
    template <typename T>
    bool operator()(const T* const camera,
                    T* residuals) const {

        T t[3], err_x;
        Eigen::Matrix4d ex = extrin;
        Eigen::Vector3d pp;
        Eigen::Vector4d cc;
        T invz, fx, fy, cx, cy;
        T ccx, ccy, ccz;
        T u, v, sdf;

        // camera[0,1,2] are the translation.
        t[0] = camera[0]; t[1] = camera[1]; t[2] = camera[2];
        err_x = T(0.0);

        // convert to camera coordinates (cc)
        cc = extrin * Eigen::Vector4d(vertex[0], vertex[1], vertex[2], 1);

        // apply our parameter offset to the extrinsics 
        ccx = T(cc(0))+t[0];
        ccy = T(cc(1))+t[1];
        ccz = T(cc(2))+t[2];
        /* project the center of the voxel onto the images */
        invz = 1.0 / ccz;
        fx = T(intrin(0, 0)); fy = T(intrin(1, 1));
        cx = T(640); cy = T(360);
//        cx = T(intrin(0, 2)); cy = T(intrin(1, 2));

        // project point p onto image
        u = ccx * fx * invz + cx ;//+ t[0];
        v = ccy * fy * invz + cy ;//+ t[1];

        // sample the image to get the sdf value 
        theInterpolator.Evaluate(v, u, &sdf);
        *residuals = (T)sdf;
        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(
        const Interpolator &theInterpolator,
        cv::Mat& theSDFimage,
        MyMesh& theMesh,
        LiveScan3d_Take & theLS3DTake,
        int cameraID) 
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 1, 3>(
            new ReprojectionError(theInterpolator, theSDFimage, theMesh, theLS3DTake.GetIntrinsics(cameraID), theLS3DTake.GetExtrinsicsInv(cameraID), cameraID)));
    }
    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* CreatePerVertex(
        const Interpolator& theInterpolator,
        const OpenMesh::Vec3f vert,
        LiveScan3d_Take& theLS3DTake,
        int cameraID)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 1, 3>(
            new ReprojectionError(theInterpolator, vert, theLS3DTake.GetIntrinsics(cameraID), theLS3DTake.GetExtrinsicsInv(cameraID), cameraID)));
    }
   // Create(theInterpolator, p, theTake, TEST_CAMERA_ID);
    // members
    cv::Mat imSDF;
    MyMesh mesh;
    int cameraID;
    Eigen::Matrix3d intrin;
    Eigen::Matrix4d extrin;
    Grid *theImageGrid;
    const Interpolator &theInterpolator;
    OpenMesh::Vec3f vertex;
};




void ProjectMeshIntoImage(MyMesh &mesh, cv::Mat &img, Eigen::Matrix3d &intrin,Eigen::Matrix4d &extrin)
{
    double t[3], err_x;
    Eigen::Matrix4d ex = extrin;
    Eigen::Vector3d pp;
    Eigen::Vector4d cc;
    double invz, fx, fy, cx, cy;
    double ccx, ccy, ccz;
    double u, v, sdf;
    int nnV = mesh.n_vertices();
    //cv::Mat testImage = cv::Mat::zeros(imDist_neg.rows, imDist_neg.cols, CV_32F);
    for (int i = 0; i < nnV; ++i)
    {
        OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(i);
        OpenMesh::Vec3f p = mesh.point(vh);
        // convert to camera coordinates (cc)
        cc = extrin * Eigen::Vector4d(p[0], p[1], p[2], 1);

        // apply our parameter offset to the extrinsics 
        ccx = double(cc(0));// + t[0];
        ccy = double(cc(1));// + t[1];
        ccz = double(cc(2));// + t[2];
        /* project the center of the voxel onto the images */
        invz = 1.0 / ccz;
        fx = (intrin(0, 0)); fy = (intrin(1, 1));
        cx = (intrin(0, 2)); cy = (intrin(1, 2));

        // project point p onto image
        u = ccx * fx * invz + cx;//+ t[0];
        v = ccy * fy * invz + cy;//+ t[1];
        if(v > 0 && v < img.rows && u > 0 && u < img.cols)
            img.at<float>(v, u) = 100;
    }
}

void MeshLab_Fix(LiveScan3d_Take &theTake, MyMesh &mesh, int CAMERAID) {

    /* this is how you compute a distance transform */
    auto imMatte = theTake.GetMatte(CAMERAID);
    cv::Mat imDist_positive, imDist_neg;// = cv::Mat::zeros(imMatte.rows, imMatte.cols, CV_8UC3);
    cv::Mat imBW;
    cv::cvtColor(imMatte, imBW, cv::COLOR_BGR2GRAY);
    cv::threshold(imBW, imBW, 40, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat imBW_inverted;
    std::string postfix = std::string("_c_" + std::to_string(CAMERAID)) + ".png";

    cv::bitwise_not(imBW, imBW_inverted);
    cv::imshow("binary image", imBW);
    //cv::imwrite("res_0_binarymatte"+postfix, imBW);

    // positive distances here are non-zero
    cv::distanceTransform(imBW_inverted, imDist_positive, cv::DIST_L2, 3, CV_32F);

    // non-zero distances here are inside object (negative distances)
    cv::distanceTransform(imBW, imDist_neg, cv::DIST_L2, 3, CV_32F);
    cv::Mat imSDF = cv::Mat::zeros(imDist_neg.rows, imDist_neg.cols, CV_32F);

    // this makes the SDF positive everywhere but decreasing towards the edges
    // zero at the edge contours
    double thresh_positive, thresh_negative;
    thresh_positive = 20;
    thresh_negative = 20;

    cv::threshold(imDist_positive, imDist_positive, thresh_positive, 100, cv::THRESH_TRUNC);
    cv::normalize(imDist_positive, imDist_positive, 0, 1.0, cv::NORM_MINMAX);
    cv::normalize(imDist_neg, imDist_neg, 0, 1.0, cv::NORM_MINMAX);
    imSDF = imDist_positive + imDist_neg;
    cv::normalize(imSDF, imSDF, 0, 1.0, cv::NORM_MINMAX);

    cv::imshow("distance", imSDF);
   // cv::imwrite("res_1_distance" + postfix, imSDF*200);
    //cv::waitKey(0);

    // ok, so one thing we have to do is convert the SDF image into a BicubicInterpolator for Ceres 
    const int cols = imSDF.cols;
    const int rows = imSDF.rows;
    EigArray img_data = EigArray(rows, cols);
    for (int j = 0; j < rows; j++)
    {
        for (int i = 0; i < cols; i++)
        {
            float sdf = imSDF.at<float>(j, i);// same as  cv::Point(i, j));
            img_data(j, i) = (double)sdf;
        }
    }
    Grid theGrid(img_data.data(), 0, img_data.rows(), 0, img_data.cols());
    Interpolator theInterpolator(theGrid);

    auto intrin = theTake.GetIntrinsics(CAMERAID);
    auto extrin = theTake.GetExtrinsicsInv(CAMERAID);
    double t[3], err_x;
    Eigen::Matrix4d ex = extrin;
    Eigen::Vector3d pp;
    Eigen::Vector4d cc;
    double invz, fx, fy, cx, cy;
    double ccx, ccy, ccz;
    double u, v, sdf;
    cv::Mat testImage = cv::Mat::zeros(imDist_neg.rows, imDist_neg.cols, CV_32F);
    ProjectMeshIntoImage(mesh, testImage, intrin, extrin);
    cv::imshow("res_2_testImage", testImage);
    cv::imwrite("res_2_meshProjection_verts" + postfix, testImage);
   // cv::waitKey(0);

    // test the ceres solver here
    ceres::Problem problem;

     // our parameter we are solving for is an "offset" to the extrinsics translation
    double x[3] = { 0.,0.,0. };
    problem.AddParameterBlock(x, 3);

    // each vertex in our mesh is an "observation" 
    // each observation requires a cost function/residual
    int nV = mesh.n_vertices();
    for (int i = 0; i < nV; ++i)
    {
        OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(i);
        OpenMesh::Vec3f p = mesh.point(vh);
        ceres::CostFunction* costPerObservation = ReprojectionError::CreatePerVertex(theInterpolator, p, theTake, CAMERAID);
        problem.AddResidualBlock(costPerObservation, nullptr, &x[0]);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // set it here for MLP
    auto eInv = theTake.GetExtrinsicsInv(CAMERAID);
    auto ee = theTake.GetExtrinsics(CAMERAID);
    std::cout << "EXTRIN BEFORE CERES:" << std::endl;
    std::cout << ee << std::endl;

    std::cout << "FINAL CERES RESULT:" << std::endl;
    // x[2] = z[0];
    std::cout << "x:" << x[0] << "," << x[1] << "," << x[2] << std::endl;
    std::cout << "extrinsics for MLP" << std::endl;


    std::cout << "BEFORE applying resulting t:" << std::endl;
    std::cout << eInv << std::endl;
    auto tt = eInv.block<3, 1>(0, 3);
    tt[0] += x[0];
    tt[1] += x[1];
    tt[2] += x[2];

    eInv.block<3, 1>(0, 3) = tt;
    std::cout << "after applying resulting t:" << std::endl;
    std::cout << eInv << std::endl;
    theTake.SetExtrinsicsInv(CAMERAID, eInv);
    // set it here for MLP
    auto e = theTake.GetExtrinsics(CAMERAID);
    e = eInv.inverse();
    std::cout << "INVERSE:" << std::endl;
    std::cout << eInv.inverse() << std::endl;

    cv::Mat testImage2 = cv::Mat::zeros(imDist_neg.rows, imDist_neg.cols, CV_32F);
    ProjectMeshIntoImage(mesh, testImage2, intrin, eInv);
    cv::imshow("testImage_AFTERCeres", testImage2);
    cv::imwrite("res_3_meshProjection_verts" + postfix, testImage2);
   // cv::waitKey(0);
}


/* THE MAIN FUNCTION */
int main(int argc, char** argv) 
{
    
    MyMesh mesh;
/********************************/
    // PARAMETERS 
//#define MESHNAME "frame_160_proc-mergedvertswithuvs.obj"
#define MESHNAME "C:\\Users\\hogue\\Desktop\\DATA\\Nov4-Take1_0\\ply\\frame_0_p2-im.obj"
#define LIVESCAN3D_TAKE_DIR "C:\\Users\\hogue\\Desktop\\DATA\\Nov4-Take1_0"
#define  MLP_OUTPUTNAME "C:\\Users\\hogue\\Desktop\\DATA\\Nov4-Take1_0\\ply\\frame_0.mlp"
    //"C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0"
    int FRAME_TO_PROCESS = 0;
    //#define MESHNAME "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_im2-remNonManFaces-cut0.obj"

    /********************************/

    if (!OpenMesh::IO::read_mesh(mesh, MESHNAME))
    {
        std::cerr << "read error" << std::endl;
        exit(1);
    }

    std::cout << "loaded: faces: " << mesh.n_faces() << std::endl;
    std::cout << "loaded: verts: " << mesh.n_vertices() << std::endl;

    LiveScan3d_Take theTake(LIVESCAN3D_TAKE_DIR, "Extrinsics_Open3D.log", 6);
    theTake.LoadFrame(0);
    
    for (int i = 0; i < 6; i++) 
    {
        std::cout << "ITERATION:CAMERAID:" << i << std::endl;
       // MeshLab_Fix(theTake, mesh, i);
    }
    SaveMLPFormat(theTake, MLP_OUTPUTNAME, MESHNAME,FRAME_TO_PROCESS);
    return 0;


    //ComputeUVs(mesh,theTake);
    mesh.triangulate();
   // PackUVs(mesh,theTake);


    // save
    OpenMesh::IO::write_mesh(mesh,
                             "C:\\Users\\hogue\\Desktop\\DATA\\aug19_hogue-rawsync_0\\ply\\frame_160_proc.obj",
                             OpenMesh::IO::Options::Flag::VertexTexCoord);

	return 0;
}