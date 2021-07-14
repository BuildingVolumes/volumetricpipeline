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



enum {
	TARGET_CHESSBOARD = 0,
	TARGET_CHESSBOARD_SB,
	TARGET_CIRCLES,
	TARGET_ACIRCLES,
	TARGET_ARUCO,
	TARGET_CHARUCO,
	TARGET_LIVESCAN,
	TARGET_APRILTAGS
};

// base class for generic calibration Target
class Target {
public:
	// rc = cols/rows (w,h)
	// sz = real world units (mm/cm etc)
	Target(cv::Size rc, cv::Size sz, int type);
	~Target();

	// override these maybe
	virtual void calculateObjectPoints() = 0;
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer) = 0;
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f>& points) = 0;
	
	// members 
	cv::Size targetSize;
	cv::Size targetSizeW;
	int type;
};

