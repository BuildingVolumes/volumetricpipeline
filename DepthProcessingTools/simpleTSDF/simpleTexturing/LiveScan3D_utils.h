#pragma once
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <k4a/k4a.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

class LiveScan3d_Client {
public:
    LiveScan3d_Client(std::string _take_dir, int clientID) : take_dir(_take_dir), id(clientID) {
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
        size_t found = 0;
        for (const auto& entry : std::filesystem::directory_iterator(client_dir)) {
            std::string p = entry.path().generic_string();
            // get frame num
           // std::cout << "p:" << p << std::endl;
            std::string pfix_color = "Color_";
            std::string pfix_depth = "Depth_";

            if ((found = p.find(pfix_color)) != std::string::npos) {
                size_t found2 = p.find(".", found + 1);
                std::string frame_s = p.substr(found + pfix_color.length(), found2 - (found + pfix_color.length()));
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
                int frame = std::atoi(p.substr(found + pfix_depth.length(), found2 - (found + pfix_depth.length())).c_str());
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
    void SetExtrinsics(Eigen::Matrix4d& m) {
        extrinsics = m;

        auto ExInv = extrinsics;
        auto R = extrinsics.block<3, 3>(0, 0);
        auto T = extrinsics.block<3, 1>(0, 3);
        auto Rt = R.transpose();
        auto T2 = -Rt * T;
        ExInv.block<3, 3>(0, 0) = Rt;
        ExInv.block<3, 1>(0, 3) = T2;


        extrinsicsInv = ExInv;// extrinsics.inverse();
        std::cout << "e:" << extrinsics << std::endl;
        std::cout << "eInv:" << extrinsicsInv << std::endl;
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
    LiveScan3d_Take(std::string _base_dir, std::string _extrinsics_name, int _numClients) : base_dir(_base_dir) {
        extrinsics_filename = base_dir + "\\" + _extrinsics_name;
        numClients = _numClients;

        for (int i = 0; i < numClients; i++) {
            LiveScan3d_Client theClient(base_dir, i);
            theClient.enumerate();
            clients.push_back(theClient);
        }
        LoadIntrinsics();
        LoadExtrinsics();
    }
    Eigen::Matrix3d& GetIntrinsics(int clientInd) {
        return clients[clientInd].GetIntrinsics();
    }
    Eigen::Matrix4d& GetExtrinsicsInv(int clientInd) {
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
