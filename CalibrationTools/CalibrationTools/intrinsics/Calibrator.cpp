#include "Calibrator.h"
Calibrator::Calibrator(std::string inputDir)
	: isCalibrated(false)
{
	this->setInputDir(inputDir);
}
Calibrator::Calibrator() : isCalibrated(false), model(NULL) {

}
Calibrator::~Calibrator() {}
void Calibrator::PrintResults()
{
	std::cout << "Calibrator: Results" << std::endl;
	cameraToCalibrate.Print();
}
bool Calibrator::setInputDir(std::string dir) {
	inputDirectory = dir;
	
	if (!fs::exists(fs::directory_entry(dir))) {
		std::cerr << "DIRECTORY DOES NOT EXIST: " << dir << std::endl;
		return false;
	}
	// list all files in the directory and add them to the inputFilenames list
	for (const auto& entry : fs::directory_iterator(dir)) {
		inputFilenames.push_back(entry.path());
	}
	return true;
}
bool Calibrator::setSelectionDir(std::string dir) {
	selectionDir = dir;
	if (!fs::exists(fs::directory_entry(dir))) {
		std::cerr << "DIRECTORY DOES NOT EXIST: " << dir << std::endl;
		return false;
	}
	return true;
}

bool Calibrator::Save(std::string fname) {
	cameraToCalibrate.Save(fname);
	return true;
}
void Calibrator::SaveSelectedImages(std::string dirName) {
	for (int i = 0; i < selectedImages.size(); i++) {
		std::string fname;
		fname = dirName + "/" + selectedFilenames[i].filename().string();
		cv::imwrite(fname, selectedImages[i]);
	}
}
using namespace lambdatwist;
bool Calibrator::RunExtrinsicsCalibration() {
		Eigen::Vector3d y1, y2, y3, X1, X2, X3;

		y1(0) = 0.0; y1(1) = 0.0; y1(2) = 1.0;
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
		}
		return true;
}
bool Calibrator::DetectTargets()
{
	int numFound = 0;

	for (int i = 0; i < this->inputFilenames.size(); i++)
	{
		std::cout << inputFilenames[i].string() << std::endl;
		cv::Mat img = cv::imread(inputFilenames[i].string());
		if (i == 0)
		{
			// get size from first image
			cameraToCalibrate.setImageSize(cv::Size(img.cols, img.rows));
		}
		bool found = this->DetectTargetsInImage(img);
		
		if (found)
		{
			selectedImages.push_back(img);
			selectedFilenames.push_back(inputFilenames[i]);
			numFound++;
		}
	}

	if (numFound == 0)
	{
		std::cout << "NO targets found" << std::endl;
		return false;
	}
	std::cout << "NUM TARGETS FOUND: "<< numFound << std::endl;
	return true;
}
