#include "CalibratorLivescan3D.h"
CalibratorLivescan3D::CalibratorLivescan3D() {}
CalibratorLivescan3D::~CalibratorLivescan3D() {}

bool CalibratorLivescan3D::DetectTargetsInImage(cv::Mat img) {
	TargetLivescan3D* theTarget = (TargetLivescan3D*)model;
	std::vector<cv::Point2f> corners;
	theTarget->calculateObjectPoints(); // do this once only

	if (theTarget->detect(img, corners, drawTarget))
	{
		allCorners.push_back(corners);
		allObjectPoints.push_back(theTarget->objectPoints);
		foundMarkers.push_back(theTarget->marker);

		return true;
	}
	return false;
}
using namespace lambdatwist;
bool CalibratorLivescan3D::RunExtrinsicsCalibration() {
	std::cout << "LS3d: extrinsics calibration " << std::endl;
	bool res = false;
	Eigen::Vector3d y1, y2, y3, X1, X2, X3;

	TargetLivescan3D* theTarget = (TargetLivescan3D*)model;
	std::vector<cv::Point3f> objPoints;
	std::vector<cv::Point2f> targetPoints;
	/*
		allObjectPoints is a vector of vector of 3d points
		this->allCorners,is a vector of vector of 2d points
		*/

	int calibrationFlags = 0;
	for (int i = 0; i < allObjectPoints.size();i++) {
		//objPoints = allObjectPoints[i];
		//targetPoints = allCorners[i];
		/*
		In objPoints for LS3D markers, we have 5 points in this order
			- TrianglePoint 0
			- topLeft 1
			- botLeft 2
			- botRight 3
			- topRight 4
		*/
		/* targetPoints contains the image points found
			- these are in the same ordering as above
		*/

		/* let's try just playing with 3 of these points and p3p()
		- restrictions: they cannot be all collinear
		- TrianglePoint, topLeft, botLeft
		*/

		/* we need the y's (image uvs) as (u,v,1) for each point */
		/* we also need the corresponding (X,Y,Z) for each point */

		y1(0) = allCorners[i][0].x-640; y1(1) = allCorners[i][0].y-360; y1(2) = 1.0;
		y2(0) = allCorners[i][1].x-640; y2(1) = allCorners[i][1].y-360; y2(2) = 1.0;
		y3(0) = allCorners[i][2].x-640; y3(1) = allCorners[i][2].y-360; y3(2) = 1.0;
		std::cout << "y1:" << y1 << std::endl;
		std::cout << "y2:" << y2 << std::endl;
		std::cout << "y3:" << y3 << std::endl;

		y1.normalize();
		y2.normalize();
		y3.normalize();
		std::cout << "y1n:" << y1 << std::endl;
		std::cout << "y2n:" << y2 << std::endl;
		std::cout << "y3n:" << y3 << std::endl;

		X1(0) = allObjectPoints[i][0].x; X1(1) = allObjectPoints[i][0].y; X1(2) = 0.01;//allObjectPoints[i][0].z;
		X2(0) = allObjectPoints[i][1].x; X2(1) = allObjectPoints[i][1].y; X2(2) = 0;//allObjectPoints[i][1].z;
		X3(0) = allObjectPoints[i][2].x; X3(1) = allObjectPoints[i][2].y; X3(2) = 0;//allObjectPoints[i][2].z;
		std::cout << "X1:" << X1 << std::endl;
		std::cout << "X2:" << X2 << std::endl;
		std::cout << "X3:" << X3 << std::endl;


		std::vector<Eigen::Vector3d> x{ y1,y2,y3 };
		std::vector<Eigen::Vector3d> X{ X1,X2,X3 };

		std::vector<CameraPose> poses;

		int numSolutions = p3p(x, X, &poses);
		std::cout << "numSolutions:" << numSolutions << std::endl;
		for (CameraPose& pose : poses) {

			double err_R = (pose.R - Eigen::Matrix3d::Identity()).norm();
			double err_t = (pose.t - Eigen::Vector3d::Zero()).norm();
			std::cout << "pose.R:" << pose.R << std::endl;
			std::cout << "pose.t:" << pose.t << std::endl;
			std::cout << "Rerr=" << err_R << " Terr=" << err_t << std::endl;
		}
	}
	

	/*y1(0) = 0.0; y1(1) = 0.0; y1(2) = 1.0;
	y2(0) = 1.0; y2(1) = 0.0; y2(2) = 1.0;
	y3(0) = 2.0; y3(1) = 1.0; y3(2) = 1.0;

	y1.normalize();
	y2.normalize();
	y3.normalize();

	X1(0) = 0.0; X1(1) = 0.0; X1(2) = 2.0;
	X2(0) = 1.41421356237309; X2(1) = 0.0; X2(2) = 1.41421356237309;
	X3(0) = 1.63299316185545; X3(1) = 0.816496580927726; X3(2) = 0.816496580927726;

	std::vector<Eigen::Vector3d> x{ y1,y2,y3 };
	std::vector<Eigen::Vector3d> X{ X1,X2,X3 };

	std::vector<CameraPose> poses;

	p3p(x, X, &poses);

	for (CameraPose& pose : poses) {

		double err_R = (pose.R - Eigen::Matrix3d::Identity()).norm();
		double err_t = (pose.t - Eigen::Vector3d::Zero()).norm();
		std::cout << "pose.R:" << pose.R << std::endl;
		std::cout << "pose.t:" << pose.t << std::endl;
		std::cout << "Rerr=" << err_R << " Terr=" << err_t << std::endl;
	}*/

	return res;
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
		std::cout << __FILE__ <<": TargetType :" << type << ": not supported" << std::endl;
	}
	return false;
}

