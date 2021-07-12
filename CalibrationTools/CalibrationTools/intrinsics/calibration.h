#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>


class Camera {
public:
	Camera() {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	}
	~Camera(){}

	//members
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	float aspectRatio;
	cv::Size imageSize;	
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;


};

namespace fs = std::filesystem;
// base class for generic calibration Target
class Target {
public:
	Target(cv::Size sz=cv::Size(9,11), cv::Size szW=cv::Size(1,1))
		: targetSize(sz)
		, targetSizeW(szW)
	{
	}

	~Target() {}
	virtual void calculateObjectPoints()=0;
	// detect the target in the given image 
	//virtual bool detectTarget(cv::Mat& img) = 0;
	cv::Size targetSize;
	cv::Size targetSizeW;
};

class TargetStandardOpenCV : public Target {
public:
	TargetStandardOpenCV(cv::Size sz = cv::Size(9, 11)) : Target(sz) {
		objectPoints.resize(1);
		objectPoints[0].resize(0);
	}
	~TargetStandardOpenCV(){}

	virtual void calculateObjectPoints() {
		objectPoints[0].resize(0);

		// this is for a chessboard only
		for (int i = 0; i < targetSize.height; i++)
			for (int j = 0; j < targetSize.width; j++)
				objectPoints[0].push_back(cv::Point3f(float(j * targetSizeW.width),
								  float(i * targetSizeW.height), 0));
		
		float grid_width = targetSizeW.width * (targetSize.width - 1);
		objectPoints[0][targetSize.width - 1].x = objectPoints[0][0].x + grid_width;

	}
	
	std::vector<std::vector<cv::Point3f> > objectPoints;
};

// types of Calibrators & Targets:
// - Standard OpenCV (chessboard, circles, assymetric circles)
// - Aruco (aruco, ChArUco)
// - Livescan (livescan marker detector )
// - AprilTag (april tag marker detector )

// base class for calibration mechanism
// contains base functionality
// derived classes maniuplate the actual marker type of detection
class Calibrator 
{
public:
	Calibrator(std::string inputDir)
		: isCalibrated(false)
	{
		this->setInputDir(inputDir);
	}
	Calibrator() : isCalibrated(false), model(NULL) {

	}
	~Calibrator() {}


	virtual bool DetectTargets()=0;
	virtual bool RunCalibration() = 0;
	virtual double ComputeAverageReprojectionError()=0;
	virtual bool setTargetInfo(cv::Size sz, std::string type)=0;

	bool setInputDir(std::string dir) {
		std::cout << "setInputDir" << std::endl;
		inputDirectory = dir;
		// list all files in the directory and add them to the inputFilenames list
		for (const auto& entry : fs::directory_iterator(dir)) {
			inputFilenames.push_back(entry.path());
		}
		return true;
	}
	bool setSelectionDir(std::string dir) {
		std::cout << "setSelectionDir" << std::endl;

		selectionDir = dir;
		return true;
	}

	bool Save(std::string fname) {
		std::cout << "SAVE" << std::endl;
		return true;
	}



	// things we need:
	// The TargetInfo
	// The input Directory
	std::string inputDirectory;
	std::vector<fs::path> inputFilenames;

	std::string selectionDir;
	
	std::vector<Target> targetsFound; // targets found in images

	// things we output:
	// calibration- intrinsics
	Camera cameraToCalibrate;
	bool isCalibrated;

	Target *model;
};

class CalibratorStandardOpenCV : public Calibrator {
public :

	CalibratorStandardOpenCV() : Calibrator() {
		calibrationFlags = 0;
		//model = new TargetStandardOpenCV();
	}
	~CalibratorStandardOpenCV() {

	}
	virtual bool DetectTargets() {
	
		std::cout << "DETECT TARGETS" << std::endl;
		for (int i = 0; i < this->inputFilenames.size(); i++) {
			std::cout << inputFilenames[i].string() << std::endl;
			cv::Mat img = cv::imread(inputFilenames[i].string());
			
			
			std::vector<cv::Point2f> pointsBuffer;
			bool found = findChessboardCorners(img, model->targetSize, pointsBuffer, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
			if (found) {
				allImagePoints.push_back(pointsBuffer);
				selectedImages.push_back(img);

				drawChessboardCorners(img, model->targetSize,cv::Mat(pointsBuffer), found);

			}
			//cv::imshow("img", img);
			//cv::waitKey(0);
		}
		return true;
	}
	virtual bool RunCalibration() {
		std::cout << "RunCalibration" << std::endl;
		/* compute object (world) points */
		std::vector<cv::Point3f> newObjPoints;
		TargetStandardOpenCV* theTarget = (TargetStandardOpenCV*)model;
		theTarget->calculateObjectPoints();
		theTarget->objectPoints.resize(allImagePoints.size(), theTarget->objectPoints[0]);

		
		double rms = calibrateCameraRO(theTarget->objectPoints, allImagePoints, this->cameraToCalibrate.imageSize, theTarget->targetSize.width - 1,
								this->cameraToCalibrate.cameraMatrix, this->cameraToCalibrate.distCoeffs, this->cameraToCalibrate.rvecs, this->cameraToCalibrate.tvecs, newObjPoints,
								calibrationFlags | cv::CALIB_FIX_K3 | cv::CALIB_USE_LU);
		printf("RMS error reported by calibrateCamera: %g\n", rms);

		return true;
	}
	virtual double ComputeAverageReprojectionError() {
		std::cout << "ComputeAverageReprojectionError()" << std::endl;
		return 0.0;
	}
	virtual bool setTargetInfo(cv::Size sz, std::string type) {
		std::cout << "setTargetInfo" << std::endl;
		model = new TargetStandardOpenCV(sz);
		return true;
	}

	std::vector< std::vector<cv::Point2f> > allImagePoints;
	std::vector< cv::Mat> selectedImages;
	int calibrationFlags;
};

