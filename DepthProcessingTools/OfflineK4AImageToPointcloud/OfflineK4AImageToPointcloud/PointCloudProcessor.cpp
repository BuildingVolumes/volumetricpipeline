#include "PointCloudProcessing.h"
#include <opencv2/opencv.hpp>
#include "tinyply.h"
#include "Utils.h"
#include "Math.h"
#include <stdio.h>
#include<fstream>


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

void PointCloudProcessing::CreatePointcloudFromK4AImage(k4a_image_t colorImage, k4a_image_t depthImage, k4a_transformation_t& transformation, std::vector<Point3f>*& outVertices, std::vector<RGB>*& outVerticeColors)
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
void PointCloudProcessing::FilterVerticesAndTransform(Point3f* vertices, RGB* color, int frameWidth, int frameHeight, std::vector<Point3f>*& outGoodVertices, std::vector<RGB>*& outGoodVertColors)
{
    //TODO: Load calibration
    Calibration calibration;

    unsigned int nVertices = frameWidth * frameHeight;

    //Reserving all the memory of the vector ahead saves some processing costs
    Point3f invalidPoint = Point3f(0, 0, 0, true);
    std::vector<Point3f> AllVertices(nVertices);
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

    std::vector<Point3f>* goodVertices = new std::vector<Point3f>(goodVerticesSize);
    std::vector<RGB>* goodColorPoints = new std::vector<RGB>(goodVerticesSize);

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

void PointCloudProcessing::WritePLY(const std::string& filename, std::vector<Point3f>* vertices, std::vector<RGB>* color)
{
    std::vector<Float3> floatVerts(vertices->size());

    for (size_t i = 0; i < vertices->size(); i++)
    {
        floatVerts[i].X = ((float)vertices->data()[i].X) / 1000;
        floatVerts[i].Y = ((float)vertices->data()[i].Y) / 1000;
        floatVerts[i].Z = ((float)vertices->data()[i].Z) / 1000;
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
