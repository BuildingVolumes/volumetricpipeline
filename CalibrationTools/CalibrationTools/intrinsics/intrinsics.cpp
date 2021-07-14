// intrinsics.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include <cxxopts.hpp>
#include <opencv2/opencv.hpp>

#include "calibration.h"

struct _ioptions{
    std::string inputDir;
    std::string outputFilename;
    std::string selectionDir;
    int target_width;
    int target_height;
    int target_rows;
    int target_cols;
    std::string target_type;
    bool saveSelected;
}ioptions;

cxxopts::ParseResult parse(int argc, char* argv[])
{
    try
    {
        cxxopts::Options options(argv[0], "Estimate the intrinsics of a single camera");

        options
            .add_options()
            ("i,inputDir", "input image directory", cxxopts::value<std::string>(ioptions.inputDir)->default_value("./input/"))
            ("o,output", "output file (yml)", cxxopts::value<std::string>(ioptions.outputFilename)->default_value("./calibration.yml"))
            ("s,selectDir", "selection dir (selected images go here)", cxxopts::value<std::string>(ioptions.selectionDir)->default_value("./selected/"))
            ("saveSelected", "", cxxopts::value <bool>(ioptions.saveSelected)->default_value("false") )
            ("r,rows", "num internal target rows", cxxopts::value<int>(ioptions.target_rows)->default_value("6"))
            ("c,cols", "num internal target columns", cxxopts::value<int>(ioptions.target_cols)->default_value("9"))
            ("W,width", "target width (mm)", cxxopts::value<int>(ioptions.target_width)->default_value("10"))
            ("H,height", "target height (mm)", cxxopts::value<int>(ioptions.target_height)->default_value("10"))
            ("t,type", "target type {chessboard, chessboardSB, circles, asymmetric_circles, aruco, charuco, livescan, apriltags}", cxxopts::value<std::string>(ioptions.target_type)->default_value("chessboard"))
            ("h,help", "print usage")
            ;
           auto result = options.parse(argc, argv);

        if (result.count("help") || result.arguments().size() == 0)
        {
            std::cout << options.help() << std::endl;
            exit(0);
        }


        return result;

    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }
}
Calibrator* CreateCalibrator() {
    std::cout << "Creating Calibrator type=" << ioptions.target_type << std::endl;
    if (ioptions.target_type.compare("chessboard") == 0)
    {
        return new CalibratorStandardOpenCV();
    }
    else if (ioptions.target_type.compare("chessboardSB") == 0) {
        return new CalibratorStandardOpenCV();

    }
    else if (ioptions.target_type.compare("circles") == 0) {
        return new CalibratorStandardOpenCV();

    }
    else if (ioptions.target_type.compare("asymmetric_circles") == 0) {
        return new CalibratorStandardOpenCV();
    }
    else if (ioptions.target_type.compare("aruco") == 0) {
        return new CalibratorAruco();
    }
    else if (ioptions.target_type.compare("charuco") == 0) {
            CalibratorAruco* arucoCalib = new CalibratorAruco();
            arucoCalib->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type, "../aruco_test/detector_params.yml");
            return arucoCalib;
    }
    else if (ioptions.target_type.compare("livescan") == 0) {
       return new CalibratorLivescan3D();
    }
    else if (ioptions.target_type.compare("apriltags") == 0) {
        return NULL;
    }
    else {
        return NULL;
    }
}

int main(int argc, char **argv)
{
    auto result = parse(argc, argv);
    auto arguments = result.arguments();

    Calibrator* calibrator = CreateCalibrator();
    if (calibrator == NULL) {
        std::cout << "Creating the Calibrator failed" << std::endl;
        return 1;
    }
  // 
  // // CalibratorStandardOpenCV calibrator;
  //  CalibratorAruco* arucoCalib = new CalibratorAruco();
  ////  arucoCalib->readDetectorParameters();
  //  arucoCalib->setInputDir(ioptions.inputDir);
  //  arucoCalib->setSelectionDir(ioptions.selectionDir);
  //  arucoCalib->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type, "../aruco_test/detector_params.yml");
    
    //CalibratorLivescan3D* livescan3Dcalib = new CalibratorLivescan3D();

    calibrator->setInputDir(ioptions.inputDir);
    calibrator->setSelectionDir(ioptions.selectionDir);
    calibrator->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type);

    if (calibrator->DetectTargets()) 
    {
        calibrator->RunCalibration();
        //double rms = calibrator->ComputeAverageReprojectionError();
        //std::cout << "RMS ERROR: " << rms << std::endl;

        calibrator->Save(ioptions.outputFilename);
        //  save 
        if (ioptions.saveSelected) {
            calibrator->SaveSelectedImages(ioptions.selectionDir);
        }
    }
}

