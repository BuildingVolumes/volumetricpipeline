#pragma once
#include "Calibrator.h"
#include "TargetStandardOpenCV.h"

class CalibratorStandardOpenCV : public Calibrator {
public:

	CalibratorStandardOpenCV();
	~CalibratorStandardOpenCV();
	virtual bool DetectTargets();
	virtual bool RunCalibration();
	virtual double ComputeAverageReprojectionError();

	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type);

	std::vector< std::vector<cv::Point2f> > allImagePoints;
	std::vector <std::vector<cv::Point3f> > allObjectPoints;

	int calibrationFlags;
};

