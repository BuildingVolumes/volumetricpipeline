#include "TargetStandardOpenCV.h"

TargetStandardOpenCV::TargetStandardOpenCV(cv::Size rc = cv::Size(9, 6), cv::Size sz = cv::Size(1, 1), int type = 0) : Target(rc, sz, type) {
	objectPoints.resize(0);
}
TargetStandardOpenCV::~TargetStandardOpenCV() {}


bool TargetStandardOpenCV::detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer) {
	bool found = false;
	switch (type) {
	case TARGET_CIRCLES:
		found = findCirclesGrid(img, targetSize, outputBuffer);
		break;
	case TARGET_CHESSBOARD:
		found = findChessboardCorners(img, targetSize, outputBuffer, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		break;
	case TARGET_CHESSBOARD_SB:
		found = findChessboardCornersSB(img, targetSize, outputBuffer, cv::CALIB_CB_ACCURACY | cv::CALIB_CB_MARKER | cv::CALIB_CB_NORMALIZE_IMAGE);
		break;
	case TARGET_ACIRCLES:
		found = findCirclesGrid(img, targetSize, outputBuffer, cv::CALIB_CB_ASYMMETRIC_GRID);
		break;
	}
	return found;
}
void TargetStandardOpenCV::draw(cv::Mat img, const std::vector<cv::Point2f>& points) {
	drawChessboardCorners(img, targetSize, cv::Mat(points), true);
}

void TargetStandardOpenCV::calculateObjectPoints() {
	/* one list of points in object/world size*/
	objectPoints.resize(0);
	switch (type) {
	case TARGET_CIRCLES:
	case TARGET_CHESSBOARD:
	case TARGET_CHESSBOARD_SB:
		for (int i = 0; i < targetSize.height; i++)
			for (int j = 0; j < targetSize.width; j++)
				objectPoints.push_back(cv::Point3f(float(j * targetSizeW.width),
									   float(i * targetSizeW.height), 0));
		break;
	case TARGET_ACIRCLES:
		for (int i = 0; i < targetSize.height; i++)
			for (int j = 0; j < targetSize.width; j++)
				objectPoints.push_back(cv::Point3f(float((2 * j + i % 2) * targetSizeW.width),
									   float(i * targetSizeW.height), 0));
		break;
	}
	float grid_width = targetSizeW.width * (targetSize.width - 1);
	objectPoints[targetSize.width - 1].x = objectPoints[0].x + grid_width;
}