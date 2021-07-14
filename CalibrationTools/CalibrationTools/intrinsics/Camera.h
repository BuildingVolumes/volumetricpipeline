#pragma once
// system headers
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

// opencv headers
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>

class Camera {
public:
	Camera() {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
		aspectRatio = 1;
		imageSize.width = 1280;
		imageSize.height = 720;
	}
	~Camera() {}

	void setImageSize(cv::Size sz) {
		imageSize = sz;
	}

	void Save(std::string fname) {
		cv::FileStorage fs(fname, cv::FileStorage::WRITE);
		fs << "image_width" << imageSize.width;
		fs << "image_height" << imageSize.height;
		fs << "camera_matrix" << cameraMatrix;
		fs << "dist_coeffs" << distCoeffs;
	}
	void Print() {
		std::cout << "---------------------------------------------------------------" << std::endl;
		std::cout << "image_width" << imageSize.width << std::endl;
		std::cout << "image_height" << imageSize.height << std::endl;
		std::cout << "camera_matrix" << cameraMatrix << std::endl;
		std::cout << "dist_coeffs" << distCoeffs << std::endl;
		std::cout << "---------------------------------------------------------------" << std::endl;
	}
	//members
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	float aspectRatio;
	cv::Size imageSize;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;


};