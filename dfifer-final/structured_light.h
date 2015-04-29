#ifndef _STRUCTURED_LIGHT_H_
#define _STRUCTURED_LIGHT_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace {

class SLPair {
 public:
	SLPair(const cv::Mat& on, const cv::Mat& off);
	int getClassification(int x, int y);
 private:
	cv::Mat _on;
	cv::Mat _off;
	cv::Mat _classification;
};

}  // namespace

class StructuredLight {
 public:
	StructuredLight(cv::Size);
	void addImagePair(const cv::Mat&, const cv::Mat&, int);
	void decode();
	cv::Mat	getColored();
	cv::Mat getCenters() const { return _centers; }
 private:
	cv::Size _dim;
	std::vector<SLPair> _v;
	std::vector<SLPair> _h;
	cv::Mat _projId;
	cv::Mat _centers;
};

#endif  // _STRUCTURED_LIGHT_H_
