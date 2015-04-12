#include "rectify.h"

Rectification::Rectification(int w, int h)
{
	_grid = cv::Size(w, h);
	_innerGrid = cv::Size(w-2, h-2);
}

bool
Rectification::rectify(const cv::Mat& input, cv::Mat& out)
{
	std::vector<cv::Point2f> corners;
	std::vector<cv::Point2f> gridPts;


	cv::Mat gray, hsv;
	cv::cvtColor(input, gray, CV_BGR2GRAY);
	cv::imwrite("gray.jpg", gray);
	
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::imwrite("hsv.jpg", hsv);
		
	bool found = cv::findChessboardCorners(input, _innerGrid, corners,
		CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
	
	if (!found) {
		return false;
	}
	

	cv::cornerSubPix(gray, corners, _innerGrid, cv::Size(-1,-1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cv::Mat markedImg = input.clone();
	cv::drawChessboardCorners(markedImg, _innerGrid, cv::Mat(corners), found);
	cv::imwrite("found.jpg", markedImg);

	cv::Point2f a = corners[1] - corners[0];
	cv::Point2f b = corners[_grid.width] - corners[0];

	// Look at sign of z in the cross product for the corner orientation
	if (a.x*b.y - a.y*b.x < 0) {
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
	
	cv::Size dsize((_grid.width-1)*100, (_grid.height-1)*100);
	cv::Mat r(dsize, input.type());

	cv::warpPerspective(input, r, t, dsize);
	out = r;

	cv::cvtColor(r, hsv, CV_BGR2Lab);
	cv::imwrite("hsv.jpg", hsv);

}
