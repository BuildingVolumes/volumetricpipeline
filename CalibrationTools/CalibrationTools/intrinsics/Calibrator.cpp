#include "Calibrator.h"
Calibrator::Calibrator(std::string inputDir)
	: isCalibrated(false)
{
	this->setInputDir(inputDir);
}
Calibrator::Calibrator() : isCalibrated(false), model(NULL) {

}
Calibrator::~Calibrator() {}

bool Calibrator::setInputDir(std::string dir) {
	std::cout << "setInputDir" << std::endl;
	inputDirectory = dir;
	// list all files in the directory and add them to the inputFilenames list
	for (const auto& entry : fs::directory_iterator(dir)) {
		inputFilenames.push_back(entry.path());
	}
	return true;
}
bool Calibrator::setSelectionDir(std::string dir) {
	std::cout << "setSelectionDir" << std::endl;

	selectionDir = dir;
	return true;
}

bool Calibrator::Save(std::string fname) {
	std::cout << "SAVE" << std::endl;
	cameraToCalibrate.Save(fname);
	return true;
}
void Calibrator::SaveSelectedImages(std::string dirName) {
	std::cout << "saving selected images to: " << dirName << std::endl;
	for (int i = 0; i < selectedImages.size(); i++) {
		std::string fname;
		fname = dirName + "/" + selectedFilenames[i].filename().string();
		std::cout << "SAVING:" << fname << std::endl;
		cv::imwrite(fname, selectedImages[i]);
	}
}

