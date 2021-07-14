#include "CalibratorLivescan3D.h"
CalibratorLivescan3D::CalibratorLivescan3D() {}
CalibratorLivescan3D::~CalibratorLivescan3D() {}

bool CalibratorLivescan3D::DetectTargets() 
{
	return false;
}
bool CalibratorLivescan3D::DetectTargetsInImage(cv::Mat img) {
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
	return false;
}


