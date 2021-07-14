#pragma once
#include "Target.h"

class TargetAruco : public Target {
public:
	TargetAruco(cv::Size rc, cv::Size sz, int type, cv::Ptr<cv::aruco::Dictionary> _dictionary, cv::Ptr<cv::aruco::DetectorParameters> _detectorParams);
	TargetAruco(cv::Size rc, cv::Size sz, int type);
	~TargetAruco();

	virtual void calculateObjectPoints();
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer);
	bool detect(cv::Mat img, std::vector<std::vector<cv::Point2f>>& outputBuffer, std::vector< int >& idBuffer, std::vector<std::vector<cv::Point2f>>& rejectedBuffer);
	
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f>& points);
	void draw(cv::Mat img, const std::vector<std::vector<cv::Point2f>>& points, std::vector< int >& idBuffer);

	// members 
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
	cv::Ptr<cv::aruco::Board> board;
	cv::Ptr<cv::aruco::GridBoard> gridboard;       // regular aruco
	cv::Ptr<cv::aruco::CharucoBoard> charucoBoard; // for charuco

};
