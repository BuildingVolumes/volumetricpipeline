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
