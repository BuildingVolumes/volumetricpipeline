#include "CalibratorStandardOpenCV.h"

CalibratorStandardOpenCV::CalibratorStandardOpenCV() : Calibrator() {
	calibrationFlags = 0;
}
CalibratorStandardOpenCV::~CalibratorStandardOpenCV() {

}
bool CalibratorStandardOpenCV::DetectTargetsInImage(cv::Mat img) {
	std::vector<cv::Point2f> pointsBuffer;
	bool found = model->detect(img, pointsBuffer);
	if (found) {
		allImagePoints.push_back(pointsBuffer);
		if(drawTarget) model->draw(img, pointsBuffer);
		return true;
	}

	return false;
}

bool CalibratorStandardOpenCV::RunCalibration() {
	std::vector<cv::Point3f> newObjPoints;
	TargetStandardOpenCV* theTarget = (TargetStandardOpenCV*)model;
	theTarget->calculateObjectPoints();

	allObjectPoints.resize(allImagePoints.size(), theTarget->objectPoints);

	double rms = calibrateCameraRO(
		allObjectPoints,
		allImagePoints,
		this->cameraToCalibrate.imageSize,
		theTarget->targetSize.width - 1,
		this->cameraToCalibrate.cameraMatrix,
		this->cameraToCalibrate.distCoeffs,
		this->cameraToCalibrate.rvecs,
		this->cameraToCalibrate.tvecs,
		newObjPoints,
		calibrationFlags | cv::CALIB_FIX_K3 | cv::CALIB_USE_LU);
	return true;
}
double CalibratorStandardOpenCV::ComputeAverageReprojectionError() {
	/* project each point using the intrinsics */
	std::vector<cv::Point2f> projectedPoints;
	double err;
	double totalErr = 0;
	int n = 0;
	for (int i = 0; i < allObjectPoints.size(); i++) {
		cv::projectPoints(cv::Mat(allObjectPoints[i]),
						  this->cameraToCalibrate.rvecs[i],
						  this->cameraToCalibrate.tvecs[i],
						  this->cameraToCalibrate.cameraMatrix,
						  this->cameraToCalibrate.distCoeffs,
						  projectedPoints);
		err = cv::norm(cv::Mat(allImagePoints[i]), cv::Mat(projectedPoints), cv::NORM_L2);
		totalErr += err * err;
		n += allObjectPoints[i].size();
	}

	return std::sqrt(totalErr / n);
}

bool CalibratorStandardOpenCV::setTargetInfo(cv::Size rc, cv::Size sz, std::string type) {
	if (type.compare("chessboard") == 0)
	{
		model = new TargetStandardOpenCV(rc, sz, TARGET_CHESSBOARD);
	}
	else if (type.compare("chessboardSB") == 0) {
		model = new TargetStandardOpenCV(rc, sz, TARGET_CHESSBOARD_SB);
	}
	else if (type.compare("circles") == 0) {
		model = new TargetStandardOpenCV(rc, sz, TARGET_CIRCLES);
	}
	else if (type.compare("asymmetric_circles") == 0) {
		model = new TargetStandardOpenCV(rc, sz, TARGET_ACIRCLES);
	}
	else {
		std::cout << __FILE__<< ": Target type: " << type << " : not supported" << std::endl;
	}
	return true;
}
