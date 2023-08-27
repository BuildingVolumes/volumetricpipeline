#pragma once
#include <vector>

typedef struct RGB
{
    char   rgbBlue;
    char    rgbGreen;
    char    rgbRed;
    char    rgbReserved;
} RGB;

class Calibration
{
public:
    std::vector<float> worldT;
    std::vector<std::vector<float>> worldR;
    int iUsedMarkerId;
    bool bCalibrated;
    Calibration();
    ~Calibration();
};

Calibration::Calibration() {};
Calibration::~Calibration() {};