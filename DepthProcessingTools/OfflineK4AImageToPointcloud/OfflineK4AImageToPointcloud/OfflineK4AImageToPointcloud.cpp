// OfflineK4AImageToPointcloud.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include<fstream>
#include <k4a/k4a.h>
//#include <k4a/k4atypes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include "tinyply.h"
using namespace std;

typedef struct Point3f
{
    Point3f()
    {
        this->X = 0;
        this->Y = 0;
        this->Z = 0;
        this->Invalid = false;
    }
    Point3f(float X, float Y, float Z, bool invalid)
    {
        this->X = X;
        this->Y = Y;
        this->Z = Z;
        this->Invalid = invalid;
    }
    Point3f(float X, float Y, float Z)
    {
        this->X = X;
        this->Y = Y;
        this->Z = Z;
        this->Invalid = false;
    }
    float X;
    float Y;
    float Z;
    bool Invalid = false;

} Point3f;

struct float3 { float x, y, z; };

typedef struct Point3s
{
    Point3s()
    {
        this->X = 0;
        this->Y = 0;
        this->Z = 0;
    }
    Point3s(short X, short Y, short Z)
    {
        this->X = X;
        this->Y = Y;
        this->Z = Z;
    }
    //meters to milimeters
    Point3s(Point3f& other)
    {
        this->X = static_cast<short>(1000 * other.X);
        this->Y = static_cast<short>(1000 * other.Y);
        this->Z = static_cast<short>(1000 * other.Z);
    }
    short X;
    short Y;
    short Z;
} Point3s;

typedef struct RGB
{
    char   rgbBlue;
    char    rgbGreen;
    char    rgbRed;
    char    rgbReserved;
} RGB;



Point3f RotatePoint(Point3f& point, std::vector<std::vector<float>>& R)
{
    Point3f res;

    res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
    res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
    res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

    return res;
}

Point3f InverseRotatePoint(Point3f& point, std::vector<std::vector<float>>& R)
{
    Point3f res;

    res.X = point.X * R[0][0] + point.Y * R[1][0] + point.Z * R[2][0];
    res.Y = point.X * R[0][1] + point.Y * R[1][1] + point.Z * R[2][1];
    res.Z = point.X * R[0][2] + point.Y * R[1][2] + point.Z * R[2][2];

    return res;
}

class Calibration
{
public:
    vector<float> worldT;
    vector<vector<float>> worldR;
    int iUsedMarkerId;
    bool bCalibrated;
    Calibration();
    ~Calibration();
};

Calibration::Calibration() {};
Calibration::~Calibration() {};

class PointCloudProcessing {

public:
    PointCloudProcessing();
    ~PointCloudProcessing();

    k4a_calibration_t GetCalibrationFromFile(std::string pathToTakeFolder);

    bool GetK4AImageFromFile(std::string path, k4a_image_t& k4aImageHandle);
    void ConvertDepthToCameraSpacePC(Point3f* pCameraSpacePoints, k4a_image_t& depthImage, int colorHeight, int colorWidth, k4a_transformation_t transformation);
    void FilterVerticesAndTransform(Point3f* vertices, RGB* colorInDepth, int frameWidth, int frameHeight, vector<Point3f>* &outGoodVertices, vector<RGB>* &outGoodVertColors);
    void WritePLY(const std::string& filename, vector<Point3f>* vertices, vector<RGB>* color);
    void CreatePointcloudFromK4AImage(k4a_image_t colorImage, k4a_image_t depthImage, k4a_transformation_t &transformation, vector<Point3f>* &outVertices, vector<RGB>* &outVerticeColors);
};

int main()
{
    PointCloudProcessing pointcloudProcessor;

    k4a_calibration_t calibration = pointcloudProcessor.GetCalibrationFromFile("");
    k4a_transformation_t transformation = k4a_transformation_create(&calibration);

    vector<Point3f>* vertices;
    vector<RGB>* verticeColors;

    k4a_image_t colorImage = NULL;
    k4a_image_t depthImage = NULL;

    if (!pointcloudProcessor.GetK4AImageFromFile("color_test.jpg", colorImage))
        std::cout << "Could not read color image" << std::endl;

    if (!pointcloudProcessor.GetK4AImageFromFile("depth_test_original.tiff", depthImage))
        std::cout << "Could not read depth image" << std::endl;

    pointcloudProcessor.CreatePointcloudFromK4AImage(colorImage, depthImage, transformation, vertices, verticeColors);
    pointcloudProcessor.WritePLY("Pointcloud", vertices, verticeColors);

    delete vertices;
    delete verticeColors;
    k4a_image_release(colorImage);
    k4a_image_release(depthImage);
    k4a_transformation_destroy(transformation);    
}

PointCloudProcessing::PointCloudProcessing() {}

void PointCloudProcessing::CreatePointcloudFromK4AImage(k4a_image_t colorImage, k4a_image_t depthImage, k4a_transformation_t &transformation, vector<Point3f>* &outVertices, vector<RGB>* &outVerticeColors) 
{
    int colorWidth = k4a_image_get_width_pixels(colorImage);
    int colorHeight = k4a_image_get_height_pixels(colorImage);

    Point3f* pointCloudInCameraCoordinates = new Point3f[colorHeight * colorWidth];
    ConvertDepthToCameraSpacePC(pointCloudInCameraCoordinates, depthImage, colorHeight, colorWidth, transformation);

    RGB* pColorRGBX = new RGB[colorHeight * colorWidth];
    memcpy(pColorRGBX, k4a_image_get_buffer(colorImage), colorHeight * colorWidth * sizeof(RGB));

    FilterVerticesAndTransform(pointCloudInCameraCoordinates, pColorRGBX, colorWidth, colorHeight, outVertices, outVerticeColors);

    delete[] pointCloudInCameraCoordinates;
    delete[] pColorRGBX;
}

//TODO: Replace fixed values with variables
k4a_calibration_t PointCloudProcessing::GetCalibrationFromFile(std::string pathToTakeFolder)
{
    //TODO: Replace with reading configuration
    cv::Mat colorImage = cv::imread("color_test.jpg");
    int colorWidth = colorImage.cols;
    int colorHeight = colorImage.rows;

    int calibrationSize;
    char* calibrationRaw;

    //TODO: What if no/one file doesn't exist?
    ifstream calibrationFile("calibration.json", ios::in | ios::binary | ios::ate);
    if (calibrationFile.is_open()) {

        calibrationSize = calibrationFile.tellg();
        calibrationRaw = new char[calibrationSize];
        calibrationFile.seekg(0, ios::beg);

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
    if (path.find(".jpg") != std::string::npos) {
        cv::Mat cvImgRaw = cv::imread(path, cv::ImreadModes::IMREAD_COLOR);
        cv::Mat cvImgBGRA = cv::Mat(cvImgRaw.rows, cvImgRaw.cols, CV_8UC4);
        cv::cvtColor(cvImgRaw, cvImgBGRA, cv::COLOR_BGR2BGRA, 4);

        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cvImgBGRA.cols, cvImgBGRA.rows, cvImgBGRA.step, &k4aImageHandle);
        memcpy(k4a_image_get_buffer(k4aImageHandle), cvImgBGRA.data, cvImgBGRA.step * cvImgBGRA.rows);
        return true;
    }

    else if (path.find(".tiff") != std::string::npos) {
        cv::Mat cvImgDepth = cv::imread(path, cv::ImreadModes::IMREAD_ANYDEPTH);
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, cvImgDepth.cols, cvImgDepth.rows, cvImgDepth.step[0], &k4aImageHandle);
        memcpy(k4a_image_get_buffer(k4aImageHandle), cvImgDepth.data, cvImgDepth.step[0] * cvImgDepth.rows);

        return true;
    }

    else return false;

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
void PointCloudProcessing::FilterVerticesAndTransform(Point3f* vertices, RGB* color, int frameWidth, int frameHeight, vector<Point3f>* &outGoodVertices, vector<RGB>*&outGoodVertColors)
{
    //TODO: Load calibration
    Calibration calibration;

    unsigned int nVertices = frameWidth * frameHeight;

    //Reserving all the memory of the vector ahead saves some processing costs
    Point3f invalidPoint = Point3f(0, 0, 0, true);
    vector<Point3f> AllVertices(nVertices);
    int goodVerticesSize = 0;

    for (unsigned int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
    {
        //All invalid vertices have a Z-Depth of 0
        if (vertices[vertexIndex].Z >= 0.0001)
        {
            Point3f temp = vertices[vertexIndex];
            RGB tempColor = color[vertexIndex];
            if (calibration.bCalibrated)
            {
                temp.X += calibration.worldT[0];
                temp.Y += calibration.worldT[1];
                temp.Z += calibration.worldT[2];
                temp = RotatePoint(temp, calibration.worldR);

                //TODO: Reimplement bound checking
                /*if (temp.X < m_vBounds[0] || temp.X > m_vBounds[3]
                    || temp.Y < m_vBounds[1] || temp.Y > m_vBounds[4]
                    || temp.Z < m_vBounds[2] || temp.Z > m_vBounds[5])
                {
                    AllVertices[vertexIndex] = invalidPoint;
                    continue;
                }*/

            }

            AllVertices[vertexIndex] = temp;
            goodVerticesSize++;
        }

        else
        {
            AllVertices[vertexIndex] = invalidPoint;
        }
    }

    vector<Point3f>* goodVertices = new vector<Point3f>(goodVerticesSize);
    vector<RGB>* goodColorPoints = new vector<RGB>(goodVerticesSize);

    int goodVerticesCounter = 0;

    //Copy all valid vertices into a clean vector 
    for (unsigned int i = 0; i < AllVertices.size(); i++)
    {
        if (!AllVertices[i].Invalid)
        {
            goodVertices->data()[goodVerticesCounter] = AllVertices[i];
            goodColorPoints->data()[goodVerticesCounter] = color[i];
            goodVerticesCounter++;
        }
    }

    //TODO: Reimplement NN-Filtering

    /*if (m_bFilter)
        filter(goodVertices, goodColorPoints, m_nFilterNeighbors, m_fFilterThreshold);*/

    //TODO: Float-To-Short Conversion somewhere else

    /*vector<Point3s> goodVerticesShort(goodVertices.size());

    for (size_t i = 0; i < goodVertices.size(); i++)
    {
        goodVerticesShort[i] = goodVertices[i];
    }*/

    outGoodVertices = goodVertices;
    outGoodVertColors = goodColorPoints;
}

void PointCloudProcessing::WritePLY(const std::string& filename, vector<Point3f>* vertices, vector<RGB>* color)
{
    vector<float3> floatVerts(vertices->size());

    for (size_t i = 0; i < vertices->size(); i++)
    {
        floatVerts[i].x = ((float)vertices->data()[i].X) / 1000;
        floatVerts[i].y = ((float)vertices->data()[i].Y) / 1000;
        floatVerts[i].z = ((float)vertices->data()[i].Z) / 1000;
    }

    /*cout << (int)color->data()[(1280 * 720) - 1].rgbRed << endl;
    cout << (int)color->data()[(1280 * 720) - 1].rgbGreen << endl;
    cout << (int)color->data()[(1280 * 720) - 1].rgbBlue << endl;*/

    std::filebuf fb_binary;
    fb_binary.open(filename + "-binary.ply", std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + filename);

    std::filebuf fb_ascii;
    fb_ascii.open(filename + "-ascii.ply", std::ios::out);
    std::ostream outstream_ascii(&fb_ascii);
    if (outstream_ascii.fail()) throw std::runtime_error("failed to open " + filename);

    tinyply::PlyFile cube_file;

    cube_file.add_properties_to_element("vertex", { "x", "y", "z" },
        tinyply::Type::FLOAT32, floatVerts.size(), reinterpret_cast<uint8_t*>(floatVerts.data()), tinyply::Type::INVALID, 0);

    cube_file.add_properties_to_element("vertex", { "blue", "green", "red", "alpha" },
        tinyply::Type::UINT8, floatVerts.size(), reinterpret_cast<uint8_t*>(color->data()), tinyply::Type::INVALID, 0);

    cube_file.get_comments().push_back("generated by tinyply 2.3");

     //Write an ASCII file
    cube_file.write(outstream_ascii, false);

     //Write a binary file
    cube_file.write(outstream_binary, true);
}

PointCloudProcessing::~PointCloudProcessing(){}


