#include "chessboard.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(a,l,h) MIN(MAX(a,l),h)

Chessboard::Chessboard(cv::Mat input)
{

	_img = input.clone();
	cv::Mat m = _img;
	
	cv::Size patternSize(6, 6);
	std::vector<cv::Point2f> corners;

	cv::Mat lab, contrast, gray;
	cv::cvtColor(m, lab, CV_BGR2Lab);
	cv::imwrite("lab.jpg", lab);
	
	contrast = m.clone();
	for (int x = 0; x < contrast.cols; ++x) {
		for (int y = 0; y < contrast.rows; ++y) {
			for (int z = 0; z < 3; ++z) {
				int c = lab.at<cv::Vec3b>(y,x)[0];
				c = 2 * (c-64)+64;
				contrast.at<cv::Vec3b>(y,x)[z] = CLAMP(c,0,255);
			}
		}
	}
	cv::imwrite("contrast.jpg", contrast);
	_img = contrast.clone();
	
	cv::cvtColor(contrast, gray, CV_BGR2GRAY);
	cv::imwrite("gray.jpg", gray);
	
	bool found = cv::findChessboardCorners(gray, patternSize, corners,
		CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);

	_cornersDetected = found;

	if (found) {
		cv::Mat gray;
		cv::cvtColor(m, gray, CV_BGR2GRAY);

		cv::cornerSubPix(gray, corners, cv::Size(6,6), cv::Size(-1,-1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		
		cv::drawChessboardCorners(m, patternSize, cv::Mat(corners), found);

		cv::imwrite("found.jpg", m);

		std::vector<std::vector<cv::Point3f> > objectPoints (1);
		std::vector<std::vector<cv::Point2f> > imagePoints (1);
		
		cv::Point2f a = corners[1] - corners[0];
		cv::Point2f b = corners[6] - corners[0];
		// Look at sign of z in the cross product for the corner orientation
		if (a.x*b.y - a.y*b.x < 0) {
			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 6; ++j) {
					cv::Point3f p(25.4*j, -25.4*i, 0.0);
					_worldCorners.push_back(p);
				}
			}
		} else {
			for (int j = 0; j < 6; ++j) {
				for (int i = 0; i < 6; ++i) {
					cv::Point3f p(25.4*(5-j), -25.4*i, 0.0);
					_worldCorners.push_back(p);
				}
			}
		}
		_imgCorners = corners;
	}
}
