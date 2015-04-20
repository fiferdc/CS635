#include "rectify.h"

Rectification::Rectification(int w, int h)
{
	_grid = cv::Size(w, h);
	_innerGrid = cv::Size(w-2, h-2);
}


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(a,l,h) MIN(MAX(a,l),h)


bool
Rectification::rectify(const cv::Mat& input, Chessboard c, cv::Mat& out)
{
	std::vector<cv::Point2f> corners;
	std::vector<cv::Point2f> gridPts;

	cv::Mat gray, hsv, lab, yuv;
	cv::Mat contrast;

	cv::cvtColor(input, lab, CV_BGR2Lab);
	cv::imwrite("lab.jpg", lab);

	cv::cvtColor(input, yuv, CV_BGR2YUV);
	cv::imwrite("yuv.jpg", yuv);
	
	contrast = input.clone();
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
	
	cv::cvtColor(contrast, gray, CV_BGR2GRAY);
//	cv::threshold(gray, gray, 50, 255, 0);
	cv::imwrite("gray.jpg", gray);

	bool found = cv::findChessboardCorners(gray, _innerGrid, corners,
		CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
	
	if (!found) {
		return false;
	}
	

	cv::cornerSubPix(gray, corners, _innerGrid, cv::Size(-1,-1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cv::Mat markedImg = input.clone();
	cv::drawChessboardCorners(markedImg, _innerGrid, cv::Mat(corners), found);
	cv::imwrite("found.jpg", markedImg);

	cv::Point2f va = corners[1] - corners[0];
	cv::Point2f vb = corners[_grid.width] - corners[0];

	// Look at sign of z in the cross product for the corner orientation
	if (va.x*vb.y - va.y*vb.x < 0) {
		for (int y = 0; y < _innerGrid.height; ++y) {
			for (int x = 0; x < _innerGrid.width; ++x) {
				gridPts.push_back(cv::Point2f(100.0*(x+1), 100.0*(y+1)));
			}
		}
	} else {
		for (int x = 0; x < _innerGrid.width; ++x) {
			for (int y = 0; y < _innerGrid.height; ++y) {
				gridPts.push_back(cv::Point2f(100.0*(_innerGrid.width-x), 100.0*(y+1)));
			}
		}
	}

	cv::Mat t = cv::findHomography(corners, gridPts);
	cv::Mat t_inv = cv::findHomography(gridPts, corners);
	
	cv::Size dsize((_grid.width-1)*100, (_grid.height-1)*100);
	cv::Mat r(dsize, input.type());

	cv::warpPerspective(input, r, t, dsize);
	out = r;
	_transformation = t_inv;
	return true;
}

cv::Point2f
Rectification::invert(cv::Point2f p)
{
	cv::Mat A(3,1,CV_64F);
	A.at<double>(0,0) = p.x;
	A.at<double>(1,0) = p.y;
	A.at<double>(2,0) = 1.0;
	cv::Mat B = _transformation * (A);
	double x = B.at<double>(0,0);
	double y = B.at<double>(1,0);
	double z = B.at<double>(2,0);
	return cv::Point2f(x/z, y/z);
}
