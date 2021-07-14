#include "CalibratorLivescan3D.h"
CalibratorLivescan3D::CalibratorLivescan3D() {}
CalibratorLivescan3D::~CalibratorLivescan3D() {}

bool CalibratorLivescan3D::DetectTargetsInImage(cv::Mat img) {
	TargetLivescan3D* theTarget = (TargetLivescan3D*)model;
	std::vector<cv::Point2f> corners;
	if (theTarget->detect(img, corners))
	{
		return true;
	}
	return false;
}

bool CalibratorLivescan3D::RunCalibration()
{
	return false;
}
double CalibratorLivescan3D::ComputeAverageReprojectionError()
{
	return 0;
}
bool CalibratorLivescan3D::setTargetInfo(cv::Size rc, cv::Size sz, std::string type)
{
	if (type.compare("livescan") == 0) {
		model = new TargetLivescan3D(rc, sz, TARGET_LIVESCAN);
	}
	else {
		std::cout << "TargetType: " << type << ": not supported" << std::endl;
	}
	return false;
}


