#ifndef _RECTIFY_H_
#define _RECTIFY_H_

#include "chessboard.h"
#include <opencv2/opencv.hpp>

class Rectification {
 public:
	Rectification(const cv::Mat&, const cv::Mat&);
	cv::Mat rectify();
	cv::Mat applyRectification(const cv::Mat&);
	std::vector<cv::Point2f> getPatPts() { return _patPts; }
	std::vector<cv::Point2f> getImgPts() { return _imgPts; }
 private:
	cv::Mat _pat;
	cv::Mat _img;
	std::vector<cv::Point2f> _patPts;
	std::vector<cv::Point2f> _imgPts;
	cv::Mat _hPatToPat;
	cv::Mat _hPatToImg;
	cv::Mat _hImgToPat1;
	cv::Mat _hImgToPat2;
};

#endif  // _RECTIFY_H_
