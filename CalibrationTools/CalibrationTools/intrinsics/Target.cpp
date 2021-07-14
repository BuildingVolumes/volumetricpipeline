#include "Target.h"

Target::Target(cv::Size rc = cv::Size(9, 6), cv::Size sz = cv::Size(1, 1), int type = 0)
	: targetSize(rc)
	, targetSizeW(sz), type(type)
{
}
Target::~Target() {}