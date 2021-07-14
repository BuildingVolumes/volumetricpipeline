#include "TargetLivescan3D.h"
TargetLivescan3D::TargetLivescan3D(cv::Size rc, cv::Size sz, int type) :Target(rc, sz, type) 
{ 
}

TargetLivescan3D::~TargetLivescan3D() 
{
}

void TargetLivescan3D::calculateObjectPoints() 
{

}
bool TargetLivescan3D::detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer)
{
	MarkerInfo marker;
	bool found = this->detector.GetMarker(img, marker);
	outputBuffer = marker.corners;
	return found;
}
bool detect(cv::Mat img, std::vector<std::vector<cv::Point2f>>& outputBuffer, std::vector< int >& idBuffer, std::vector<std::vector<cv::Point2f>>& rejectedBuffer)
{
	return false;
}

void TargetLivescan3D::draw(cv::Mat img, const std::vector<cv::Point2f>& points)
{

}
void TargetLivescan3D::draw(cv::Mat img, const std::vector<std::vector<cv::Point2f>>& points, std::vector< int >& idBuffer) 
{

}