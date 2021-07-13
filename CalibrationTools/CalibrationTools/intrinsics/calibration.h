#pragma once
// system headers
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

// opencv headers
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>


enum {
	TARGET_CHESSBOARD = 0,
	TARGET_CIRCLES,
	TARGET_ACIRCLES,
	TARGET_ARUCO,
	TARGET_CHARUCO,
	TARGET_LIVESCAN,
	TARGET_APRILTAGS
};

class Camera {
public:
	Camera() {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
		aspectRatio = 1;
		imageSize.width = 1280;
		imageSize.height = 720;
	}
	~Camera(){}

	void setImageSize(cv::Size sz) {
		imageSize = sz;
	}

	void Save(std::string fname) {
		cv::FileStorage fs(fname, cv::FileStorage::WRITE);
		fs << "image_width" << imageSize.width;
		fs << "image_height" << imageSize.height;
		fs << "camera_matrix" << cameraMatrix;
		fs << "dist_coeffs" << distCoeffs;
	}
	//members
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	float aspectRatio;
	cv::Size imageSize;	
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;


};

namespace fs = std::filesystem;
// base class for generic calibration Target
class Target {
public:
	// rc = cols/rows (w,h)
	// sz = real world units (mm/cm etc)
	Target(cv::Size rc = cv::Size(9,6), cv::Size sz=cv::Size(1,1), int type=0)
		: targetSize(rc)
		, targetSizeW(sz), type(type)
	{
	}

	~Target() {}
	virtual void calculateObjectPoints()=0;
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f> &outputBuffer) = 0;
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f> &points)=0;
	//drawChessboardCorners(img, model->targetSize,cv::Mat(pointsBuffer), found);

	// detect the target in the given image 
	//virtual bool detectTarget(cv::Mat& img) = 0;
	cv::Size targetSize;
	cv::Size targetSizeW;
	int type;
};
class TargetAruco : public Target {
public:
	TargetAruco(cv::Size rc, cv::Size sz, int type, cv::Ptr<cv::aruco::Dictionary> _dictionary, cv::Ptr<cv::aruco::DetectorParameters> _detectorParams)
		: Target(rc,sz,type)
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
	TargetAruco(cv::Size rc = cv::Size(9, 6), cv::Size sz = cv::Size(1, 1), int type = 0)
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
	~TargetAruco(){}
	virtual void calculateObjectPoints() {

	}
	virtual bool detect(cv::Mat img, std::vector<cv::Point2f>& outputBuffer) {
		// use other one or call it from here somehow
		return false;
	}
	bool detect(cv::Mat img, std::vector<std::vector<cv::Point2f>>& outputBuffer, std::vector< int > &idBuffer, std::vector<std::vector<cv::Point2f>>& rejectedBuffer)
	{
		//cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

		cv::aruco::detectMarkers(img, board->dictionary, outputBuffer,idBuffer, detectorParams, rejectedBuffer);
		cv::aruco::refineDetectedMarkers(img, board, outputBuffer, idBuffer, rejectedBuffer);
		return true;
	}
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f>& points) {
		// not used, each type is going to need its own interface.... uggggh... should get rid of the generic one then
	}
	void draw(cv::Mat img, const std::vector<std::vector<cv::Point2f>>& points, std::vector< int >& idBuffer) {
		if(idBuffer.size() > 0) cv::aruco::drawDetectedMarkers(img, points, idBuffer);
		else {
			std::cout << "ARUCO: draw: no ids to draw" << std::endl;
		}
	}

	// members 
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
	cv::Ptr<cv::aruco::Board> board;
	cv::Ptr<cv::aruco::GridBoard> gridboard;       // regular aruco
	cv::Ptr<cv::aruco::CharucoBoard> charucoBoard; // for charuco

};

class TargetStandardOpenCV : public Target {
public:
	TargetStandardOpenCV(cv::Size rc = cv::Size(9, 6), cv::Size sz = cv::Size(1,1), int type=0) : Target(rc, sz,type) {
		objectPoints.resize(0);
	}
	~TargetStandardOpenCV(){}

	virtual bool detect(cv::Mat img, std::vector<cv::Point2f> &outputBuffer) {
		bool found=false;
		switch (type) {
		case TARGET_CIRCLES:
			found = findCirclesGrid(img, targetSize, outputBuffer);
			break;
		case TARGET_CHESSBOARD:
			found = findChessboardCorners(img, targetSize, outputBuffer, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
			break;
		case TARGET_ACIRCLES:
			found = findCirclesGrid(img, targetSize, outputBuffer, cv::CALIB_CB_ASYMMETRIC_GRID);
			break;
		}
		return found;
	}
	virtual void draw(cv::Mat img, const std::vector<cv::Point2f> &points) {
		drawChessboardCorners(img, targetSize,cv::Mat(points), true);
	}
	virtual void calculateObjectPoints() {
		/* one list of points in object/world size*/
		objectPoints.resize(0);
		switch (type) {
		case TARGET_CIRCLES:
		case TARGET_CHESSBOARD:
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
	
	std::vector<cv::Point3f> objectPoints;
};

// types of Calibrators & Targets:
// - Standard OpenCV (chessboard, circles, assymetric circles)
// - Aruco (aruco, ChArUco)
// - Livescan (livescan marker detector )
// - AprilTag (april tag marker detector )

// base class for calibration mechanism
// contains base functionality
// derived classes maniuplate the actual marker type of detection
class Calibrator 
{
public:
	Calibrator(std::string inputDir)
		: isCalibrated(false)
	{
		this->setInputDir(inputDir);
	}
	Calibrator() : isCalibrated(false), model(NULL) {

	}
	~Calibrator() {}


	virtual bool DetectTargets()=0;
	virtual bool RunCalibration() = 0;
	virtual double ComputeAverageReprojectionError()=0;
	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type)=0;

	bool setInputDir(std::string dir) {
		std::cout << "setInputDir" << std::endl;
		inputDirectory = dir;
		// list all files in the directory and add them to the inputFilenames list
		for (const auto& entry : fs::directory_iterator(dir)) {
			inputFilenames.push_back(entry.path());
		}
		return true;
	}
	bool setSelectionDir(std::string dir) {
		std::cout << "setSelectionDir" << std::endl;

		selectionDir = dir;
		return true;
	}

	bool Save(std::string fname) {
		std::cout << "SAVE" << std::endl;
		cameraToCalibrate.Save(fname);
		return true;
	}
	void SaveSelectedImages(std::string dirName) {
		std::cout << "saving selected images to: " << dirName << std::endl;
		for (int i = 0; i < selectedImages.size(); i++) {
			std::string fname;
			fname = dirName +"/" + selectedFilenames[i].filename().string();
			std::cout << "SAVING:"<< fname << std::endl;
			cv::imwrite(fname, selectedImages[i]);
		}
	}


	// things we need:
	// The TargetInfo
	// The input Directory
	std::string inputDirectory;
	std::vector<fs::path> inputFilenames;

	std::string selectionDir;
	
	std::vector<Target> targetsFound; // targets found in images

	// things we output:
	// calibration- intrinsics
	Camera cameraToCalibrate;
	bool isCalibrated;
	std::vector< cv::Mat> selectedImages;
	std::vector<fs::path> selectedFilenames;
	Target *model;
};

class CalibratorAruco : public Calibrator {
public:
	CalibratorAruco() {}
	~CalibratorAruco() {}

	/**
 */
	static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters>& params) {
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
			return false;
		fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
		fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
		fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
		fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
		fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
		fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
		fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
		fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
		fs["minDistanceToBorder"] >> params->minDistanceToBorder;
		fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
		fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
		fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
		fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
		fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
		fs["markerBorderBits"] >> params->markerBorderBits;
		fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
		fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
		fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
		fs["minOtsuStdDev"] >> params->minOtsuStdDev;
		fs["errorCorrectionRate"] >> params->errorCorrectionRate;
		return true;
	}

	virtual bool DetectTargets() {
		std::cout << "DETECT TARGETS" << std::endl;
		int numFound = 0;
		TargetAruco* theTarget = (TargetAruco*)model;
		for (int i = 0; i < this->inputFilenames.size(); i++) {
			std::cout << inputFilenames[i].string() << std::endl;
			cv::Mat img = cv::imread(inputFilenames[i].string());
			if (i == 0) {
				// get size from first image
				cameraToCalibrate.setImageSize(cv::Size(img.cols, img.rows));
			}
			std::vector< int > ids;
			std::vector< std::vector<cv::Point2f >> pointsBuffer, rejected;
			
			bool found = theTarget->detect(img, pointsBuffer, ids, rejected);


			if (found) {

				if (theTarget->type == TARGET_CHARUCO) {
					// interpolate charuco corners
					cv::Mat currentCharucoCorners, currentCharucoIds;
					if (ids.size() > 0)
						cv::aruco::interpolateCornersCharuco(pointsBuffer, ids, img, theTarget->charucoBoard, currentCharucoCorners, currentCharucoIds);

					if (currentCharucoCorners.total() > 0)
						cv::aruco::drawDetectedCornersCharuco(img, currentCharucoCorners, currentCharucoIds);
				}
				else {
					theTarget->draw(img, pointsBuffer, ids);
				}
				allCorners.push_back(pointsBuffer);
				allIds.push_back(ids);
				selectedImages.push_back(img);
				selectedFilenames.push_back(inputFilenames[i]);
				//cv::imshow("charuco", img);
				//cv::waitKey();
				numFound++;
			}
		}


		if (numFound == 0) {
			std::cout << "NO targets found" << std::endl;
			return false;
		}
		std::cout << "Num Targets Found:" << numFound << std::endl;
		return true;
		return false;
	}
	virtual bool RunCalibration() {
		TargetAruco* theTarget = (TargetAruco*)model;
		std::vector< std::vector< cv::Point2f > > allCornersConcatenated;
		std::vector< int > allIdsConcatenated;
		std::vector< int > markerCounterPerFrame;
		markerCounterPerFrame.reserve(allCorners.size());
		for (unsigned int i = 0; i < allCorners.size(); i++) {
			markerCounterPerFrame.push_back((int)allCorners[i].size());
			for (unsigned int j = 0; j < allCorners[i].size(); j++) {
				allCornersConcatenated.push_back(allCorners[i][j]);
				allIdsConcatenated.push_back(allIds[i][j]);
			}
		}
		// might have to pass these in as before in the parent
		int calibrationFlags = 0;

		// calibrate camera
		cv::Ptr<cv::aruco::Board> board = theTarget->board;
		double repError = -1;
	
		repError = cv::aruco::calibrateCameraAruco(
							allCornersConcatenated,
							allIdsConcatenated,
							markerCounterPerFrame,
							board,
							cameraToCalibrate.imageSize,
							cameraToCalibrate.cameraMatrix,
							cameraToCalibrate.distCoeffs,
							cameraToCalibrate.rvecs,
							cameraToCalibrate.tvecs,
							calibrationFlags);
		if (theTarget->type == TARGET_CHARUCO)
		{
			// prepare data for charuco calibration
			int nFrames = (int)allCorners.size();
			std::vector< cv::Mat > allCharucoCorners;
			std::vector< cv::Mat > allCharucoIds;
			std::vector< cv::Mat > filteredImages;
			allCharucoCorners.reserve(nFrames);
			allCharucoIds.reserve(nFrames);
			for (int i = 0; i < nFrames; i++) {
				// interpolate using camera parameters
				cv::Mat currentCharucoCorners, currentCharucoIds;
				cv::aruco::interpolateCornersCharuco(
							allCorners[i], 
							allIds[i], 
							selectedImages[i], 
							theTarget->charucoBoard,
							currentCharucoCorners, 
							currentCharucoIds, 
							cameraToCalibrate.cameraMatrix,
							cameraToCalibrate.distCoeffs);

				allCharucoCorners.push_back(currentCharucoCorners);
				allCharucoIds.push_back(currentCharucoIds);
				filteredImages.push_back(selectedImages[i]);
			}
			/*if (allCharucoCorners.size() < 4) {
				std::cerr << "Not enough corners for calibration" << std::endl;
				return 0;
			}*/
			repError =
				cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, 
												  theTarget->charucoBoard, 
												  cameraToCalibrate.imageSize,
												  cameraToCalibrate.cameraMatrix, 
												  cameraToCalibrate.distCoeffs, 
												  cameraToCalibrate.rvecs, 
												  cameraToCalibrate.tvecs, 
												  calibrationFlags);
		}
		
		

		std::cout << "ARUCO: repError = " << repError << std::endl;
		return false;
	}
	virtual double ComputeAverageReprojectionError() {
		return -1;
	}
	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type) {
		return this->setTargetInfo(rc, sz, type, "detector.params");
	}
	bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type, std::string detectorParamsFile = "detector.params") {

		if (type.compare("aruco") == 0) {
			std::cout << "Target type: " << type << std::endl;
			detectorParams = cv::aruco::DetectorParameters::create();
			bool readOk = readDetectorParameters(detectorParamsFile, detectorParams);
			if (!readOk) {
				std::cerr << "ARUCO: Invalid detector parameters file" << std::endl;
				return 0;
			}

			// make this a parameter
			int dictionaryId = cv::aruco::DICT_6X6_250;
			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
			// do stuff here
			/* create the board */
			model = new TargetAruco(rc,sz,TARGET_ARUCO, dictionary, detectorParams);

		}
		else if (type.compare("charuco") == 0) {
			std::cout << "Target type: " << type << std::endl;
			detectorParams = cv::aruco::DetectorParameters::create();
			bool readOk = readDetectorParameters(detectorParamsFile, detectorParams);
			if (!readOk) {
				std::cerr << "CHARUCO: Invalid detector parameters file" << std::endl;
				return 0;
			}

			// make this a parameter
			int dictionaryId = cv::aruco::DICT_6X6_250;
			dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
			// do stuff here
			/* create the board */
			model = new TargetAruco(rc, sz, TARGET_CHARUCO, dictionary, detectorParams);

		}
		else {

		}
		return true;
	}
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	// collected frames for calibration
	std::vector< std::vector< std::vector< cv::Point2f > > > allCorners;
	std::vector< std::vector< int > > allIds;

};


class CalibratorStandardOpenCV : public Calibrator {
public :

	CalibratorStandardOpenCV() : Calibrator() {
		calibrationFlags = 0;
	}
	~CalibratorStandardOpenCV() {

	}
	virtual bool DetectTargets() {
	
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
	virtual bool RunCalibration() {
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
	virtual double ComputeAverageReprojectionError() {
		std::cout << "ComputeAverageReprojectionError::" << std::endl;
		/* project each point using the intrinsics */
		std::vector<cv::Point2f> projectedPoints;
		double err;
		double totalErr = 0;
		int n=0;
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

		return std::sqrt(totalErr/n);
	}

	virtual bool setTargetInfo(cv::Size rc, cv::Size sz, std::string type) {
		//   ("t,type", "target type {chessboard, circles, asymmetric_circles, aruco, charuco, livescan, apriltags}
		if (type.compare("chessboard")==0) 
		{
			model = new TargetStandardOpenCV(rc, sz, TARGET_CHESSBOARD);
		}
		else if (type.compare("circles")==0) {
			model = new TargetStandardOpenCV(rc, sz,TARGET_CIRCLES);
		}
		else if (type.compare("asymmetric_circles")==0) {
			model = new TargetStandardOpenCV(rc, sz, TARGET_ACIRCLES);
		}
		else {
			std::cout << "Target type: " << type << " : not supported" << std::endl;
		}
		
		

		return true;
	}

	std::vector< std::vector<cv::Point2f> > allImagePoints;
	std::vector <std::vector<cv::Point3f> > allObjectPoints;
	
	int calibrationFlags;
};

