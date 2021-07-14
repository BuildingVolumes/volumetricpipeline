#pragma once
#include "Target.h"
#include "marker.h"

class TargetLivescan3D : public Target
{
public:
	TargetLivescan3D(cv::Size rc, cv::Size sz, int type);
	~TargetLivescan3D();

	virtual void calculateObjectPoints();
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer);
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer, bool drawTarget);
	//virtual bool detect(cv::Mat img, std::vector<std::vector<cv::Point2f>>& outputBuffer, std::vector< int >& idBuffer, std::vector<std::vector<cv::Point2f>>& rejectedBuffer);

	virtual void draw(cv::Mat img, const std::vector<cv::Point2f>& points);
	void draw(cv::Mat img, const std::vector<std::vector<cv::Point2f>>& points, std::vector< int >& idBuffer);

	MarkerInfo marker;
	std::vector<cv::Point3f> objectPoints; // canonical marker points (5 corners)
	MarkerDetector detector;
};

