#pragma once

#include <k4a/k4a.h>
#include <string>
#include <vector>
#include "Math.h"
#include <filesystem>
#include "Utils.h"


class PointCloudProcessing
{

public:
    PointCloudProcessing();
    ~PointCloudProcessing();

    k4a_calibration_t GetCalibrationFromFile(std::string pathToTakeFolder, std::string pathToColorImage);

    bool GetK4AImageFromFile(std::string path, k4a_image_t& k4aImageHandle);
    void TransformDepthToColorAndSave(std::vector<std::string> colorFiles, std::vector<std::string> depthFiles, k4a_transformation_t transformation, std::string savePath);
    void ConvertDepthToCameraSpacePC(Point3f* pCameraSpacePoints, k4a_image_t& depthImage, int colorHeight, int colorWidth, k4a_transformation_t transformation);
    void FilterVerticesAndTransform(Point3f* vertices, RGB* colorInDepth, int frameWidth, int frameHeight, std::vector<Point3f>*& outGoodVertices, std::vector<RGB>*& outGoodVertColors);
    void WritePLY(const std::string& filename, std::vector<Point3f>* vertices, std::vector<RGB>* color);
    std::vector<std::filesystem::path> GetClientPathsFromTakePath(std::string takepath);
    void CreatePointcloudFromK4AImage(k4a_image_t colorImage, k4a_image_t depthImage, k4a_transformation_t& transformation, std::vector<Point3f>*& outVertices, std::vector<RGB>*& outVerticeColors);
    k4a_image_t TransformDepthToColor(k4a_image_t& depthImage, int colorHeight, int colorWidth, k4a_transformation_t transformation);

};