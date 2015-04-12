#ifndef _PROJ_DETECTOR_H_
#define _PROJ_DETECTOR_H_

#include "line.h"

#include <opencv2/opencv.hpp>
#include <vector>

bool
FindLines(const cv::Mat& in, std::vector<Line>& out);

void
FindIntersections(const std::vector<Line>& h, const std::vector<Line>& v, std::vector<cv::Point2f>& ret);

#endif  // _PROJ_DETECTOR_H_
