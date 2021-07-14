#pragma once
#include "Calibrator.h"
#include "TargetLiveScan3D.h"

class CalibratorLivescan3D :
    public Calibrator
{
public:
	CalibratorLivescan3D();
	~CalibratorLivescan3D();

	virtual bool DetectTargetsInImage(cv::Mat img);
	virtual bool RunCalibration();
	virtual double ComputeAverageReprojectionError();
	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type);

	std::vector<std::vector<cv::Point2f>> allCorners;
	std::vector<std::vector<cv::Point3f>> allObjectPoints;
	std::vector<MarkerInfo> foundMarkers;
};

