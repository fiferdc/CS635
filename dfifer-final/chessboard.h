#ifndef _CHESSBOARD_H_
#define _CHESSBOARD_H_

#include <opencv2/opencv.hpp>
#include <vector>

class Chessboard {
 public:
	Chessboard(cv::Mat m);
	std::vector<cv::Point2f> getImgCorners() const {
		return _imgCorners;
	}
	std::vector<cv::Point3f> getWorldCorners() const {
		return _worldCorners;
	}
	cv::Mat getSrcImg() const {
		return _img;
	}
	bool cornersDetected() const {
		return _cornersDetected; 
	}
 private:
	std::vector<cv::Point2f> _imgCorners;
	std::vector<cv::Point3f> _worldCorners;
	cv::Mat _img;
	bool _cornersDetected;
};

#endif  // _CHESSBOARD_H_
