#include "PointCloudProcessing.h"
#include <opencv2/opencv.hpp>
#include "tinyply.h"
#include "Utils.h"
#include "Math.h"
#include <stdio.h>
#include<fstream>
#include "vmath.hpp/vmath_all.hpp"

using fvec3 = vmath_hpp::vec<float, 3>;

PointCloudProcessing::PointCloudProcessing()
{
}

PointCloudProcessing::~PointCloudProcessing()
{
}

std::vector<std::filesystem::path> PointCloudProcessing::GetClientPathsFromTakePath(std::string takepath)
{
    std::vector<std::filesystem::path> clientPaths;

    for (const auto& entry : std::filesystem::directory_iterator(takepath))
    {
        if (entry.path().string().find("client_") != std::string::npos)
        {
            clientPaths.push_back(entry.path());
        }
    }

    return clientPaths;
}

int PointCloudProcessing::GetIDFromPath(std::string path)
{
    std::vector<std::string> splittedStrings = SplitString(path, '_');
    return std::stoi(splittedStrings[splittedStrings.size() - 1]);
}

int PointCloudProcessing::GetIndexFromColorFileName(std::string path)
{
    std::vector<std::string> splittedStrings = SplitString(path, '_');
    splittedStrings = SplitString(splittedStrings[splittedStrings.size() - 1], '.');
    return std::stoi(splittedStrings[0]);
}

Matrix4x4 PointCloudProcessing::LoadOpen3DExtrinsics(const int clientNumber, std::filesystem::path pathToCapture)
{
    std::ifstream file;
    Matrix4x4 extrinsic = Matrix4x4::GetIdentity();

    std::filesystem::path filePath = pathToCapture / "Extrinsics_Open3D.log";
    file.open(filePath.string());
    if (!file.is_open())
    {
        std::cout << "Could not open extrinsics: " + filePath.string() << std::endl;
        return extrinsic;
    }

    std::string serial;
    int number = -1;
    int notUsed;

    while (number != clientNumber)
    {
        file >> notUsed;
        file >> serial;
        file >> number;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                file >> extrinsic.mat[i][j];
            }
        }
    }
        

    if (number == -1)
    {
        std::cout << "Could not find extrinsic client: " + std::to_string(clientNumber) << std::endl;
        return extrinsic;
    }

    else
    {
        std::cout << "Found extrinsics for client : " + std::to_string(number) << std::endl;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                std::cout << extrinsic.mat[i][j] << " ";
            }

            std::cout << std::endl;
        }
    }

        return extrinsic;
}


//TODO: Replace fixed values with variables
k4a_calibration_t PointCloudProcessing::GetCalibrationFromFile(std::string pathToTakeFolder, std::string pathToColorImage)
{
    //TODO: Replace with reading configuration
    cv::Mat colorImage = cv::imread(pathToColorImage);
    int colorWidth = colorImage.cols;
    int colorHeight = colorImage.rows;

    int calibrationSize;
    char* calibrationRaw;

    std::string pathToCalibFile = "";

    for (const auto& entry : std::filesystem::directory_iterator(pathToTakeFolder))
    {
        if (entry.path().string().find(".json") != std::string::npos)
            pathToCalibFile = entry.path().string();
    }

    //TODO: What if no/one file doesn't exist?
    std::ifstream calibrationFile(pathToCalibFile, std::ios::in | std::ios::binary | std::ios::ate);
    if (calibrationFile.is_open())
    {

        calibrationSize = calibrationFile.tellg();
        calibrationRaw = new char[calibrationSize];
        calibrationFile.seekg(0, std::ios::beg);

        calibrationFile.read(calibrationRaw, calibrationSize);
        calibrationFile.close();

        k4a_calibration_t loadedCalibration;
        k4a_calibration_get_from_raw(calibrationRaw, calibrationSize, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P, &loadedCalibration);
        delete[] calibrationRaw;

        return loadedCalibration;
    }
}

/// <summary>
/// Opens a file on disk and reads it into a k4a_image_t. Only .jpg pictures for color and .tiff pictures for depth are currently supported.
/// Returns a BGRA Image for color files and a DEPTH16 image for depth files.
/// </summary>
/// <param name="path"></param>
/// <param name="imageFormat"></param>
/// <param name="k4aImageHandle"></param>
/// <returns></returns>
bool PointCloudProcessing::GetK4AImageFromFile(std::string path, k4a_image_t& k4aImageHandle)
{
    if (path.find(".jpg") != std::string::npos)
    {
        cv::Mat cvImgRaw = cv::imread(path, cv::ImreadModes::IMREAD_COLOR);
        cv::Mat cvImgBGRA = cv::Mat(cvImgRaw.rows, cvImgRaw.cols, CV_8UC4);
        cv::cvtColor(cvImgRaw, cvImgBGRA, cv::COLOR_BGR2BGRA, 4);

        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cvImgBGRA.cols, cvImgBGRA.rows, cvImgBGRA.step, &k4aImageHandle);
        memcpy(k4a_image_get_buffer(k4aImageHandle), cvImgBGRA.data, cvImgBGRA.step * cvImgBGRA.rows);
        return true;
    }

    else if (path.find(".tiff") != std::string::npos)
    {
        cv::Mat cvImgDepth = cv::imread(path, cv::ImreadModes::IMREAD_ANYDEPTH);
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, cvImgDepth.cols, cvImgDepth.rows, cvImgDepth.step[0], &k4aImageHandle);
        memcpy(k4a_image_get_buffer(k4aImageHandle), cvImgDepth.data, cvImgDepth.step[0] * cvImgDepth.rows);

        return true;
    }

    else return false;

}

void PointCloudProcessing::TransformDepthToColorAndSave(std::vector<std::string> colorFiles, std::vector<std::string> depthFiles, k4a_transformation_t transformation, std::string savePath)
{
    PointCloudProcessing pointcloudProcessor;
    k4a_image_t colorImage = NULL;
    k4a_image_t depthImage = NULL;

    for (size_t i = 0; i < colorFiles.size(); i++)
    {
        std::cout << colorFiles[i] << std::endl;
        std::cout << depthFiles[i] << std::endl;

        std::string splitter = "_";
        std::string fileNumber = colorFiles[i].substr(colorFiles[i].find(splitter) + 1, (colorFiles[i].find(".") - colorFiles[i].find(splitter)) - 1);

        if (!pointcloudProcessor.GetK4AImageFromFile(colorFiles[i], colorImage))
            std::cout << "Could not read color image" << std::endl;

        if (!pointcloudProcessor.GetK4AImageFromFile(depthFiles[i], depthImage))
            std::cout << "Could not read depth image" << std::endl;

        int colorWidth = k4a_image_get_width_pixels(colorImage);
        int colorHeight = k4a_image_get_height_pixels(colorImage);

        k4a_image_t transformedImage = pointcloudProcessor.TransformDepthToColor(depthImage, colorHeight, colorWidth, transformation);
        cv::Mat transformedDepthImage = cv::Mat(k4a_image_get_height_pixels(transformedImage), k4a_image_get_width_pixels(transformedImage), CV_16UC1, k4a_image_get_buffer(transformedImage), k4a_image_get_stride_bytes(transformedImage));

        std::string outPutPath = savePath + "TransformedOutput/" + "Transformed_Image_" + fileNumber + ".tiff";
        cv::imwrite(outPutPath, transformedDepthImage);
        std::cout << "Wrote file: " + outPutPath << std::endl;

        k4a_image_release(transformedImage);
        k4a_image_release(colorImage);
        k4a_image_release(depthImage);
    }
}

/// <summary>
/// Projects a 2D Depth Map into a 3D Pointcloud in Camera Space Coordinates. Does not yet apply the marker calibration offset
/// </summary>
/// <param name="pCameraSpacePoints"></param>
/// <param name="depthImage"></param>
/// <param name="colorHeight"></param>
/// <param name="colorWidth"></param>
/// <param name="transformation"></param>
void PointCloudProcessing::ConvertDepthToCameraSpacePC(Point3f* pCameraSpacePoints, k4a_image_t& depthImage, int colorHeight, int colorWidth, k4a_transformation_t transformation)
{
    k4a_image_t pointCloudImage = NULL;
    k4a_image_t transformedDepthImage = NULL;

    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth, colorHeight, colorWidth * sizeof(uint16_t), &transformedDepthImage);
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth, colorHeight, colorWidth * 3 * (int)sizeof(int16_t), &pointCloudImage);

    k4a_result_t depthToColorResult = k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformedDepthImage);
    k4a_result_t depthToPointCloudResult = k4a_transformation_depth_image_to_point_cloud(transformation, transformedDepthImage, K4A_CALIBRATION_TYPE_COLOR, pointCloudImage);

    int16_t* pointCloudData = (int16_t*)k4a_image_get_buffer(pointCloudImage);

    for (int i = 0; i < colorHeight; i++)
    {
        for (int j = 0; j < colorWidth; j++)
        {
            pCameraSpacePoints[j + i * colorWidth].X = pointCloudData[3 * (j + i * colorWidth) + 0] / 1000.0f;
            pCameraSpacePoints[j + i * colorWidth].Y = pointCloudData[3 * (j + i * colorWidth) + 1] / 1000.0f;
            pCameraSpacePoints[j + i * colorWidth].Z = pointCloudData[3 * (j + i * colorWidth) + 2] / 1000.0f;
        }
    }

    k4a_image_release(transformedDepthImage);
    k4a_image_release(pointCloudImage);
}

k4a_image_t PointCloudProcessing::TransformDepthToColor(k4a_image_t& depthImage, int colorHeight, int colorWidth, k4a_transformation_t transformation)
{
    k4a_image_t transformedDepthImage = NULL;

    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth, colorHeight, colorWidth * sizeof(uint16_t), &transformedDepthImage);

    k4a_result_t depthToColorResult = k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformedDepthImage);

    return transformedDepthImage;
}

void PointCloudProcessing::CreatePointcloudFromK4AImage(k4a_image_t colorImage, k4a_image_t depthImage, k4a_transformation_t transformation, Matrix4x4 extrinsics, std::vector<Point3f>*& outVertices, std::vector<RGB>*& outVerticeColors)
{
    int colorWidth = k4a_image_get_width_pixels(colorImage);
    int colorHeight = k4a_image_get_height_pixels(colorImage);

    Point3f* pointCloudInCameraCoordinates = new Point3f[colorHeight * colorWidth];
    ConvertDepthToCameraSpacePC(pointCloudInCameraCoordinates, depthImage, colorHeight, colorWidth, transformation);

    RGB* pColorRGBX = new RGB[colorHeight * colorWidth];
    memcpy(pColorRGBX, k4a_image_get_buffer(colorImage), colorHeight * colorWidth * sizeof(RGB));

    FilterVerticesAndTransform(pointCloudInCameraCoordinates, pColorRGBX, colorWidth, colorHeight, extrinsics, outVertices, outVerticeColors);

    delete[] pointCloudInCameraCoordinates;
    delete[] pColorRGBX;
}

/// <summary>
/// Filters out any invalid vertices, and optionally also with Nearest Neighbor-Filtering.
/// Applies the transformation offset given by the extrinsic marker calibration.
/// Don't forget to delete the outVertices & outVerticesColors after using them.
/// </summary>
/// <param name="vertices"></param>
/// <param name="colorInDepth"></param>
/// <param name="frameWidth"></param>
/// <param name="frameHeight"></param>
/// <param name="outGoodVertices"></param>
/// <param name="outGoodVerticesSize"></param>
/// <param name="outGoodVertColors"></param>
void PointCloudProcessing::FilterVerticesAndTransform(Point3f* vertices, RGB* color, int frameWidth, int frameHeight, Matrix4x4 extrinsics, std::vector<Point3f>*& outGoodVertices, std::vector<RGB>*& outGoodVertColors)
{
    unsigned int nVertices = frameWidth * frameHeight;

    //Reserving all the memory of the vector ahead saves some processing costs
    Point3f invalidPoint = Point3f(0, 0, 0, true);
    std::vector<Point3f> AllVertices(nVertices);
    int goodVerticesSize = 0;

    Matrix4x4 scale = Matrix4x4(
        0.001f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.001f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.001f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);

    Matrix4x4 toWorld = extrinsics;

    //Bounding Box values
    fvec3 center(0,0,0); // Center of the box.
    fvec3 dx(-0.958, - 0.059, 0.279);
    fvec3 dy(-0.122, 0.969, - 0.213);
    fvec3 dz(0.258, 0.238, 0.936); // X,Y, and Z directions, normalized.
    fvec3 half(2,2,2); // Box size in each dimension, divided by 2.

    for (unsigned int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
    {
        //All invalid vertices have a Z-Depth of 0
        if (vertices[vertexIndex].Z >= 0.0001)
        {
            Point3f temp = vertices[vertexIndex];
            RGB tempColor = color[vertexIndex];
            
            temp = toWorld * temp;

            fvec3 point;
            point.x = temp.X;
            point.y = temp.Y;
            point.z = temp.Z;

            //Check if point is inside bounding box
            fvec3 d = point - center;
            bool inside = vmath_hpp::abs(vmath_hpp::dot(d, dx)) <= half.x &&
                vmath_hpp::abs(vmath_hpp::dot(d, dy)) <= half.y &&
                vmath_hpp::abs(vmath_hpp::dot(d, dz)) <= half.z;

            if (inside)
            {
                AllVertices[vertexIndex] = temp;
                goodVerticesSize++;
            }

            else
                AllVertices[vertexIndex] = invalidPoint;
        }

        else
            AllVertices[vertexIndex] = invalidPoint;

    }

    outGoodVertices = new std::vector<Point3f>(goodVerticesSize);
    outGoodVertColors = new std::vector<RGB>(goodVerticesSize);

    int goodVerticesCounter = 0;

    //Copy all valid vertices into a clean vector 
    for (unsigned int i = 0; i < AllVertices.size(); i++)
    {
        if (!AllVertices[i].Invalid)
        {
            outGoodVertices->data()[goodVerticesCounter] = AllVertices[i];
            outGoodVertColors->data()[goodVerticesCounter] = color[i];
            goodVerticesCounter++;
        }
    }
}

void PointCloudProcessing::WritePLY(const std::string& filename, std::vector<Point3f>* vertices, std::vector<RGB>* color)
{
    std::vector<Float3> floatVerts(vertices->size());

    for (size_t i = 0; i < vertices->size(); i++)
    {
        floatVerts[i].X = ((float)vertices->data()[i].X);
        floatVerts[i].Y = ((float)vertices->data()[i].Y);
        floatVerts[i].Z = ((float)vertices->data()[i].Z);
    }

    std::filebuf fb_binary;
    fb_binary.open(filename, std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail())
    {
        std::cout << "failed to open " + filename << std::endl;
    }

    tinyply::PlyFile cube_file;

    cube_file.add_properties_to_element("vertex", { "x", "y", "z" },
        tinyply::Type::FLOAT32, floatVerts.size(), reinterpret_cast<uint8_t*>(floatVerts.data()), tinyply::Type::INVALID, 0);

    cube_file.add_properties_to_element("vertex", { "blue", "green", "red", "alpha" },
        tinyply::Type::UINT8, floatVerts.size(), reinterpret_cast<uint8_t*>(color->data()), tinyply::Type::INVALID, 0);

    cube_file.get_comments().push_back("generated by tinyply 2.3");

    //Write a binary file
    cube_file.write(outstream_binary, true);
}


std::vector<std::string> PointCloudProcessing::SplitString(std::string str, char splitter)
{
    std::vector<std::string> result;
    std::string current = "";
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == splitter)
        {
            if (current != "")
            {
                result.push_back(current);
                current = "";
            }
            continue;
        }
        current += str[i];
    }
    if (current.size() != 0)
        result.push_back(current);
    return result;
}
