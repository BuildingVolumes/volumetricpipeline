// intrinsics.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include <cxxopts.hpp>
#include <opencv2/opencv.hpp>

#include "calibration.h"

struct _ioptions{
    // file options 
    std::string inputDir;
    std::string outputFilename;
    std::string selectionDir;

    // toggles
    bool saveSelected;
    bool drawTarget;

    // target options 
    int target_width;
    int target_height;
    int target_rows;
    int target_cols;
       
    std::string target_type;
    std::string aruco_dict;
    std::string aruco_detector_params;
}ioptions;

/*
Samples command arguments:
-i ../chessboardsb_test\images -s ../selected --saveSelected -t chessboardSB -r 9 -c 14 -W 10 -H 10 -o ../chessboardsb_test/calibration.yaml
-i ../aruco_test/images -s ../selected --saveSelected --drawTarget -t aruco -r 7 -c 5 -W 10 -H 10 --dict 6x6_50 --params ../aruco_test/detector_params.yml -o ../aruco_test/calibration.yaml

*/
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
            ("drawTarget", "Draw Target boolean", cxxopts::value <bool>(ioptions.drawTarget)->default_value("false"))
            ("h,help", "print usage")
            ;
        options.add_options("Aruco")
            ("params", "Aruco Detector Parameters File (yml)", cxxopts::value<std::string>(ioptions.aruco_detector_params)->default_value("detectorDefaults.yml"))
            ("dict", "Aruco Dictionary: 4x4_50, 4x4_100, 4x4_250, 4x4_1000, etc... ", cxxopts::value<std::string>(ioptions.aruco_dict)->default_value("ORIGINAL"));

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
        CalibratorStandardOpenCV *calibrator = new CalibratorStandardOpenCV();
        calibrator->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type);
        return calibrator;
    }
    else if (ioptions.target_type.compare("chessboardSB") == 0) {
        CalibratorStandardOpenCV* calibrator = new CalibratorStandardOpenCV();
        calibrator->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type);
        return calibrator;
    }
    else if (ioptions.target_type.compare("circles") == 0) {
        CalibratorStandardOpenCV* calibrator = new CalibratorStandardOpenCV();
        calibrator->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type);
        return calibrator;
    }
    else if (ioptions.target_type.compare("asymmetric_circles") == 0) {
        CalibratorStandardOpenCV* calibrator = new CalibratorStandardOpenCV();
        calibrator->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type);
        return calibrator;
    }
    else if (ioptions.target_type.compare("aruco") == 0) {
        CalibratorAruco* arucoCalib = new CalibratorAruco();
        arucoCalib->setDictionary(ioptions.aruco_dict);
        arucoCalib->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type, ioptions.aruco_detector_params);
        return arucoCalib;
    }
    else if (ioptions.target_type.compare("charuco") == 0) {
        CalibratorAruco* arucoCalib = new CalibratorAruco();
        arucoCalib->setDictionary(ioptions.aruco_dict);
        arucoCalib->setTargetInfo(cv::Size(ioptions.target_cols, ioptions.target_rows), cv::Size(ioptions.target_width, ioptions.target_height), ioptions.target_type, ioptions.aruco_detector_params);// "../aruco_test/detector_params.yml");
        return arucoCalib;
    }
    else if (ioptions.target_type.compare("livescan") == 0) {
        CalibratorLivescan3D* calibrator = new CalibratorLivescan3D();
        return calibrator;
    }
    else if (ioptions.target_type.compare("apriltags") == 0) {
        return NULL;
    }
    else {
        return NULL;
    }
}

void PrintOptionsSelected() {
    std::cout << "===============================================================" << std::endl;
    std::cout << "Intrinsics Calibrator:" << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "Running with Options:" << std::endl;
    std::cout << left<<setw(25)<< "- InputDir: " << ioptions.inputDir << std::endl;
    std::cout << left<<setw(25)<<"- OutputFilename:" << ioptions.outputFilename << std::endl;
    std::cout << left<<setw(25)<<"- SelectionDir: " << ioptions.selectionDir << std::endl;
    std::cout << left << setw(25) << "- saveSelected: " << ioptions.saveSelected << std::endl;
    std::cout << left << setw(25) << "- drawTarget: " << ioptions.drawTarget << std::endl;
    std::cout << left<<setw(25)<<"- Target (WxH): " << "(" << ioptions.target_width << " x " << ioptions.target_height << ")" << std::endl;
    std::cout << left<<setw(25)<<"- Target (RxC): " << "(" << ioptions.target_rows << " x " << ioptions.target_cols << ")" << std::endl;
    std::cout << left<<setw(25)<<"- Target Type: " << ioptions.target_type << std::endl;
    std::cout << left << setw(25) << "- Aruco Dict: " << ioptions.aruco_dict << std::endl;
    std::cout << left << setw(25) << "- Aruco Detector Params: " << ioptions.aruco_detector_params << std::endl;
    std::cout << "===============================================================" << std::endl;
}


int main(int argc, char **argv)
{
    auto result = parse(argc, argv);
    auto arguments = result.arguments();
    PrintOptionsSelected();
    std::cout << "===============================================================" << std::endl;
   
    Calibrator* calibrator = CreateCalibrator();
    if (calibrator == NULL) {
        std::cout << "Creating the Calibrator failed" << std::endl;
        return 1;
    }
    std::cout << "STAGE: SETTING GENERIC CALIBRATOR OPTIONS" << std::endl;
    // set generic calibrator options
    if (!calibrator->setInputDir(ioptions.inputDir)) return 1;
    if (!calibrator->setSelectionDir(ioptions.selectionDir)) return 1;

    calibrator->setDrawTarget(ioptions.drawTarget);

    std::cout << "STAGE: DETECTING TARGETS" << std::endl;
    if (calibrator->DetectTargets()) 
    {
        std::cout << "STAGE: RUNNING CALIBRATION" << std::endl;
        if (calibrator->RunCalibration())
        {
            calibrator->PrintResults();
            std::cout << "STAGE: SAVING" << std::endl;
            calibrator->Save(ioptions.outputFilename);
        }
        if (ioptions.saveSelected) {
            std::cout << "STAGE: SAVING SELECTED IMAGES" << std::endl;
            calibrator->SaveSelectedImages(ioptions.selectionDir);
        }
    }
    std::cout << "===============================================================" << std::endl;
}

