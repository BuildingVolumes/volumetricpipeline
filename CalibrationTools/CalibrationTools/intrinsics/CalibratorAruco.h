#pragma once

#include "Calibrator.h"
#include "TargetAruco.h"

class CalibratorAruco : public Calibrator {
public:
	CalibratorAruco();
	~CalibratorAruco();

	static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters>& params) {
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;
		fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
		fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
		fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
		fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
		fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
		fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
		fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
		fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
		fs["minDistanceToBorder"] >> params->minDistanceToBorder;
		fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
		fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
		fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
		fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
		fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
		fs["markerBorderBits"] >> params->markerBorderBits;
		fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
		fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
		fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
		fs["minOtsuStdDev"] >> params->minOtsuStdDev;
		fs["errorCorrectionRate"] >> params->errorCorrectionRate;
		return true;
	}

	virtual bool DetectTargets();
	virtual bool DetectTargetsInImage(cv::Mat img);
	virtual bool RunCalibration();
	virtual double ComputeAverageReprojectionError();
	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type);
	bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type, std::string detectorParamsFile);

	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	// collected frames for calibration
	std::vector< std::vector< std::vector< cv::Point2f > > > allCorners;
	std::vector< std::vector< int > > allIds;

};
