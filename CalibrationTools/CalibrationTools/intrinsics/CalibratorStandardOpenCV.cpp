#include "CalibratorStandardOpenCV.h"

CalibratorStandardOpenCV::CalibratorStandardOpenCV() : Calibrator() {
	calibrationFlags = 0;
}
CalibratorStandardOpenCV::~CalibratorStandardOpenCV() {

}
bool CalibratorStandardOpenCV::DetectTargets() {

	std::cout << "DETECT TARGETS" << std::endl;
	int numFound = 0;
	for (int i = 0; i < this->inputFilenames.size(); i++) {
		std::cout << inputFilenames[i].string() << std::endl;
		cv::Mat img = cv::imread(inputFilenames[i].string());
		if (i == 0) {
			// get size from first image
			cameraToCalibrate.setImageSize(cv::Size(img.cols, img.rows));
		}
		std::vector<cv::Point2f> pointsBuffer;

		bool found = model->detect(img, pointsBuffer);
		//bool found = findChessboardCorners(img, model->targetSize, pointsBuffer, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found) {

			allImagePoints.push_back(pointsBuffer);
			selectedImages.push_back(img);
			selectedFilenames.push_back(inputFilenames[i]);
			//drawChessboardCorners(img, model->targetSize,cv::Mat(pointsBuffer), found);
			model->draw(img, pointsBuffer);
			numFound++;
		}
	}


	if (numFound == 0) {
		std::cout << "NO targets found" << std::endl;
		return false;
	}
	std::cout << "Num Targets Found:" << numFound << std::endl;
	return true;
}
bool CalibratorStandardOpenCV::RunCalibration() {
	std::cout << "RunCalibration::" << std::endl;
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
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	return true;
}
double CalibratorStandardOpenCV::ComputeAverageReprojectionError() {
	std::cout << "ComputeAverageReprojectionError::" << std::endl;
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
	//   ("t,type", "target type {chessboard, circles, asymmetric_circles, aruco, charuco, livescan, apriltags}
	if (type.compare("chessboard") == 0)
	{
		model = new TargetStandardOpenCV(rc, sz, TARGET_CHESSBOARD);
	}
	else if (type.compare("circles") == 0) {
		model = new TargetStandardOpenCV(rc, sz, TARGET_CIRCLES);
	}
	else if (type.compare("asymmetric_circles") == 0) {
		model = new TargetStandardOpenCV(rc, sz, TARGET_ACIRCLES);
	}
	else {
		std::cout << "Target type: " << type << " : not supported" << std::endl;
	}



	return true;
}