#ifndef _FEATURE_MATCHER_H_
#define _FEATURE_MATCHER_H_

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureMatcher {
 public:
	FeatureMatcher(const cv::Mat&, const cv::Mat&);
	cv::Mat match();
	std::vector<cv::Point2f> getPatFeatures();
	std::vector<cv::Point2f> getImgFeatures();
	cv::Mat getPatToImg() { return _patToImg; }
	cv::Mat getImgToPat() { return _imgToPat; }
	cv::Mat getMask()	{ return _mask; }
 private:
	cv::Mat _pat;
	cv::Mat _img;
	cv::Mat _mask;
	cv::Mat _patToImg;
	cv::Mat _imgToPat;
	std::vector<cv::KeyPoint> _patFeat;
	std::vector<cv::KeyPoint> _imgFeat;
	std::vector<cv::DMatch> _matches;
};

#endif
