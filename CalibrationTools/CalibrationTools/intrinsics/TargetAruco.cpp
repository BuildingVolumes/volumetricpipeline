#include "TargetAruco.h"

TargetAruco::TargetAruco(cv::Size rc, cv::Size sz, int type, cv::Ptr<cv::aruco::Dictionary> _dictionary, cv::Ptr<cv::aruco::DetectorParameters> _detectorParams)
	: Target(rc, sz, type)
{
	int markersX = rc.width;
	int markersY = rc.height;
	float markerLength = sz.width; // marker length
	float markerSeparation = sz.height; // marker separation
	dictionary = _dictionary;
	detectorParams = _detectorParams;

	if (type == TARGET_ARUCO) {
		gridboard = cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
		board = gridboard.staticCast<cv::aruco::Board>();
	}
	else if (type == TARGET_CHARUCO) {
		this->charucoBoard = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
		board = charucoBoard.staticCast<cv::aruco::Board>();
		// FIXME: set parameters properly
	}
}

TargetAruco::TargetAruco(cv::Size rc = cv::Size(9, 6), cv::Size sz = cv::Size(1, 1), int type = 0)
	: Target(rc, sz, type)
{
	int markersX = rc.width;
	int markersY = rc.height;
	float markerLength = sz.width; // marker length
	float markerSeparation = sz.height; // marker separation
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_ARUCO_ORIGINAL));

	if (type == TARGET_ARUCO) {
		gridboard = cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
		board = gridboard.staticCast<cv::aruco::Board>();
	}
	else if (type == TARGET_CHARUCO) {
		this->charucoBoard = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
		board = charucoBoard.staticCast<cv::aruco::Board>();
		// FIXME: set parameters properly
	}
}
TargetAruco::~TargetAruco() {}

void TargetAruco::calculateObjectPoints() {

}

bool TargetAruco::detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer) {
	// use other one or call it from here somehow
	return false;
}

bool TargetAruco::detect(cv::Mat img, std::vector<std::vector<cv::Point2f>>& outputBuffer, std::vector< int >& idBuffer, std::vector<std::vector<cv::Point2f>>& rejectedBuffer)
{
	//cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

	cv::aruco::detectMarkers(img, board->dictionary, outputBuffer, idBuffer, detectorParams, rejectedBuffer);
	if (outputBuffer.size() == 0) return false;
	cv::aruco::refineDetectedMarkers(img, board, outputBuffer, idBuffer, rejectedBuffer);
	if (idBuffer.size() == 0) return false;
	return true;
}

void TargetAruco::draw(cv::Mat img, const std::vector<cv::Point2f>& points) {
	// not used, each type is going to need its own interface.... uggggh... should get rid of the generic one then
}

void TargetAruco::draw(cv::Mat img, const cv::Mat& points,cv::Mat& idBuffer) 
{
	cv::aruco::drawDetectedCornersCharuco(img, points, idBuffer);

}
void TargetAruco::draw(cv::Mat img, const std::vector<std::vector<cv::Point2f>>& points, std::vector< int >& idBuffer) 
{
	if (idBuffer.size() > 0) cv::aruco::drawDetectedMarkers(img, points, idBuffer);
	else {
		std::cout << "ARUCO: draw: no ids to draw" << std::endl;
	}	
}