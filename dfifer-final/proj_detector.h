#ifndef _PROJ_DETECTOR_H_
#define _PROJ_DETECTOR_H_

#include "line.h"

#include <opencv2/opencv.hpp>
#include <vector>

void
relabel(float** actual, cv::Mat centers, int* relabeled, int count=6);

bool
FindLines(const cv::Mat& in, cv::Mat& clusters, std::vector<Line>& out);

void
FindIntersections(const std::vector<Line>& h, const std::vector<Line>& v, std::vector<cv::Point2f>& ret);

std::vector<cv::Point2f>
classifyPoints(
		cv::Mat h, cv::Mat v, std::vector<cv::Point2f> pts,
		const char * pattern);

#endif  // _PROJ_DETECTOR_H_
