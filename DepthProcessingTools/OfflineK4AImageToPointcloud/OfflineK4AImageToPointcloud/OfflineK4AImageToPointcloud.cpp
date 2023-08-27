// OfflineK4AImageToPointcloud.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <k4a/k4atypes.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "Math.h"
#include <filesystem>
#include "PointCloudProcessing.h"


class ClientData
{
public:
    std::filesystem::path clientPath;
    std::vector<std::string> colorFiles;
    std::vector<std::string> depthFiles;
    k4a_transformation_t transformation;
    k4a_calibration_t intrinsics;
    Matrix4x4 calibrationMatrix;
    Matrix4x4 refinementMatrix;

};



int main()
{
    std::string pathToCapture = "D:\\Test\\";


    PointCloudProcessing pcProcessor;

    std::vector<std::filesystem::path> clientPaths = pcProcessor.GetClientPathsFromTakePath(pathToCapture);

    for (size_t i = 0; i < clientPaths.size(); i++)
    {
        std::cout << clientPaths[i].string() << std::endl;
    }


    /*
    for (const auto& entry : fs::directory_iterator(pathToCapture))
    {
        if (entry.path().string().find("Color") != std::string::npos)
            colorFiles.push_back(entry.path().string());

        if (entry.path().string().find("Depth") != std::string::npos)
            depthFiles.push_back(entry.path().string());
    }

    */

    //k4a_calibration_t calibration = pointcloudProcessor.GetCalibrationFromFile(pathToCapture, colorFiles[0]);
    //k4a_transformation_t transformation = k4a_transformation_create(&calibration);

    //vector<Point3f>* vertices;
    //vector<RGB>* verticeColors;

    

    //pointcloudProcessor.CreatePointcloudFromK4AImage(colorImage, depthImage, transformation, vertices, verticeColors);
    //pointcloudProcessor.WritePLY("Pointcloud", vertices, verticeColors);

    //delete vertices;
    //elete verticeColors;
    //k4a_image_release(colorImage);
    //k4a_image_release(depthImage);
    //k4a_transformation_destroy(transformation);    
}




