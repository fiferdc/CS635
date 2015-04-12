#ifndef _RECTIFY_H_
#define _RECTIFY_H_

#include <opencv2/opencv.hpp>

class Rectification {
 public:
	Rectification(int, int);
	bool rectify(const cv::Mat&, cv::Mat&);
 private:
	cv::Size _grid;
	cv::Size _innerGrid;
	cv::Mat _transformation;
};

#endif  // _RECTIFY_H_
