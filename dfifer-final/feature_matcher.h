#ifndef _FEATURE_MATCHER_H_
#define _FEATURE_MATCHER_H_

#include <opencv2/opencv.hpp>

class FeatureMatcher {
 public:
	FeatureMatcher(const cv::Mat&, const cv::Mat&);
	cv::Mat match();
 private:
	cv::Mat _pat;
	cv::Mat _img;
};

#endif
