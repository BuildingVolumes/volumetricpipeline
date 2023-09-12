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
	int clientID;
	std::vector<std::string> colorFiles;
	std::vector<std::string> depthFiles;
	k4a_transformation_t transformation;
	k4a_calibration_t intrinsics;
	Matrix4x4 calibrationMatrix;
	Matrix4x4 refinementMatrix;

	std::vector<Point3f>* vertices = NULL;
	std::vector<RGB>* colors = NULL;
};


std::vector<ClientData> LoadClientData(std::filesystem::path pathToCapture, PointCloudProcessing pcProcessor);
void GetVerticesFromRawImages(ClientData& clientData, int imageIndex, PointCloudProcessing pcProcessor);

int main()
{
	std::string pathToCapture = "C:\\Test\\";
	PointCloudProcessing pcProcessor;
	std::vector<ClientData> clients = LoadClientData(pathToCapture, pcProcessor);

	int i = 0;

	for (size_t i = 0; i < clients[0].colorFiles.size(); i++)
	{
		std::vector<Point3f> allVertices;
		std::vector<RGB> allColors;

		for (size_t j = 0; j < clients.size(); j++)
		{
			if (clients[j].vertices != NULL)
			{
				delete clients[j].vertices;
				clients[j].vertices = NULL;
			}

			if (clients[j].colors != NULL)
			{
				delete clients[j].colors;
				clients[j].colors = NULL;
			}

			GetVerticesFromRawImages(clients[j], i, pcProcessor);

			allVertices.insert(allVertices.end(), clients[j].vertices->begin(), clients[j].vertices->end());
			allColors.insert(allColors.end(), clients[j].colors->begin(), clients[j].colors->end());
		}

		std::string filename = pathToCapture + "\\out\\video_" + std::to_string(i);
		std::cout << "Writing file: " + filename << std::endl;
		pcProcessor.WritePLY(filename, &allVertices, &allColors);


	}

}


std::vector<ClientData> LoadClientData(std::filesystem::path pathToCapture, PointCloudProcessing pcProcessor)
{
	std::vector<ClientData> clients;
	std::vector<std::filesystem::path> clientPaths = pcProcessor.GetClientPathsFromTakePath(pathToCapture.string());


	for (size_t i = 0; i < clientPaths.size(); i++)
	{
		ClientData client;
		client.clientPath = clientPaths[i];
		std::cout << "Client Path: = " + client.clientPath.string() << std::endl;

		client.clientID = pcProcessor.GetIDFromPath(client.clientPath.string());
		std::cout << "Client ID: = " + std::to_string(client.clientID) << std::endl;


		for (const auto& entry : std::filesystem::directory_iterator(client.clientPath.string()))
		{
			if (entry.path().string().find("Color") != std::string::npos)
				client.colorFiles.push_back(entry.path().string());

			if (entry.path().string().find("Depth") != std::string::npos)
				client.depthFiles.push_back(entry.path().string());
		}

		if (client.colorFiles.size() == 0 || client.depthFiles.size() == 0)
		{
			std::cout << "Color or depth images could not be found for client: " + std::to_string(i) << std::endl;
			continue;
		}

		client.intrinsics = pcProcessor.GetCalibrationFromFile(client.clientPath.string(), client.colorFiles[0]);
		client.transformation = k4a_transformation_create(&client.intrinsics);
		client.calibrationMatrix = pcProcessor.LoadOpen3DExtrinsics(client.clientID, pathToCapture);

		clients.push_back(client);
	}

	return clients;
}


void GetVerticesFromRawImages(ClientData& clientData, int imageIndex, PointCloudProcessing pcProcessor)
{
	std::string colorImagePath = clientData.colorFiles[imageIndex];
	std::string depthImagePath = clientData.depthFiles[imageIndex];

	k4a_image_t colorImage;
	k4a_image_t depthImage;

	pcProcessor.GetK4AImageFromFile(colorImagePath, colorImage);
	pcProcessor.GetK4AImageFromFile(depthImagePath, depthImage);

	pcProcessor.CreatePointcloudFromK4AImage(colorImage, depthImage, clientData.transformation, clientData.calibrationMatrix, clientData.vertices, clientData.colors);

	k4a_image_release(colorImage);
	k4a_image_release(depthImage);


}












