#include "TargetLivescan3D.h"
TargetLivescan3D::TargetLivescan3D(cv::Size rc, cv::Size sz, int type) :Target(rc, sz, type) 
{ 
}

TargetLivescan3D::~TargetLivescan3D() 
{
}

void TargetLivescan3D::calculateObjectPoints() 
{
	if (objectPoints.size() > 0) return;
	// 5 points in canonical frame
	// width/height specified in target in real world coords
	// center needs to be computed and then offset
	cv::Point3f center(this->targetSizeW.width/2.f, this->targetSizeW.height/2.f,0);
	// 0,h // tl
	// 0,0 // bl
	// w,0 // br
	// w,h // tr

	// triangle point
	// 0,h*0.60
	float ratio = 0.6f;
	cv::Point3f trianglePoint(0, this->targetSizeW.height * ratio,0);
	cv::Point3f topLeft(0,this->targetSizeW.height,0);
	cv::Point3f botLeft(0, 0, 0);
	cv::Point3f botRight(this->targetSizeW.width,0,0);
	cv::Point3f topRight(this->targetSizeW.width, this->targetSizeW.height,0);

	// center them
	trianglePoint -= center;
	topLeft -= center;
	botLeft -= center;
	botRight -= center;
	topRight -= center;

	// the first point is the triangle point
	this->objectPoints.push_back(trianglePoint);
	// the rest are the square coord boundaries
	this->objectPoints.push_back(topLeft);
	this->objectPoints.push_back(botLeft);
	this->objectPoints.push_back(botRight);
	this->objectPoints.push_back(topRight);

}

bool TargetLivescan3D::detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer)
{
	bool found = this->detector.GetMarker(img, marker);

	std::cout << "detect:marker num corners: " << marker.corners.size() << std::endl;

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