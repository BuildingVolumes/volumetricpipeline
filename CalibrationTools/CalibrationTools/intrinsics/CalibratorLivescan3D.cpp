#include "CalibratorLivescan3D.h"
CalibratorLivescan3D::CalibratorLivescan3D() {}
CalibratorLivescan3D::~CalibratorLivescan3D() {}

bool CalibratorLivescan3D::DetectTargetsInImage(cv::Mat img) {
	TargetLivescan3D* theTarget = (TargetLivescan3D*)model;
	std::vector<cv::Point2f> corners;
	theTarget->calculateObjectPoints(); // do this once only

	if (theTarget->detect(img, corners))
	{
		allCorners.push_back(corners);
		allObjectPoints.push_back(theTarget->objectPoints);
		foundMarkers.push_back(theTarget->marker);

		return true;
	}
	return false;
}

bool CalibratorLivescan3D::RunCalibration()
{
	TargetLivescan3D* theTarget = (TargetLivescan3D*)model;
	std::vector<cv::Point3f> newObjPoints;
	int calibrationFlags = 0;
	// allCorners are the corners of all markers 
	// allObjectPoints are the corresponding canonical object points 
	double rms = calibrateCameraRO(
		allObjectPoints,
		this->allCorners,
		this->cameraToCalibrate.imageSize,
		theTarget->targetSize.width - 1,
		this->cameraToCalibrate.cameraMatrix,
		this->cameraToCalibrate.distCoeffs,
		this->cameraToCalibrate.rvecs,
		this->cameraToCalibrate.tvecs,
		newObjPoints,
		calibrationFlags | cv::CALIB_FIX_K3 | cv::CALIB_USE_LU);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

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


