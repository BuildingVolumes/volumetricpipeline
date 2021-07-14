#pragma once
#include "Target.h"

class TargetStandardOpenCV : public Target {
public:
	TargetStandardOpenCV(cv::Size rc, cv::Size sz, int type);
	~TargetStandardOpenCV();

	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer);
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f>& points);
	virtual void calculateObjectPoints();

	std::vector<cv::Point3f> objectPoints;
};
