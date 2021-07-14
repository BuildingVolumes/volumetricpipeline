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

#include "Target.h"
#include "Camera.h"

namespace fs = std::filesystem;

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
	Calibrator(std::string inputDir);
	Calibrator();
	~Calibrator();

	// override these for each type of calibrator
	virtual bool DetectTargets() = 0;
	virtual bool RunCalibration() = 0;
	virtual double ComputeAverageReprojectionError() = 0;
	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type) = 0;

	// base generic functionality
	bool setInputDir(std::string dir);
	bool setSelectionDir(std::string dir);
	bool Save(std::string fname);
	void SaveSelectedImages(std::string dirName);


	// things most calibrators need:
	std::string inputDirectory;
	std::vector<fs::path> inputFilenames;
	std::string selectionDir;
	std::vector<Target> targetsFound; // targets found in images

	// things we output:
	// calibration- intrinsics
	Camera cameraToCalibrate;
	bool isCalibrated;
	std::vector< cv::Mat> selectedImages;
	std::vector<fs::path> selectedFilenames;
	Target* model;
};
