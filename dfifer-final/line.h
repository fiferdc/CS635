#ifndef _LINE_H_
#define _LINE_H_
#include <opencv2/opencv.hpp>

// Defines a line from the line segment (a,b)
class Line {
 public:
	Line(cv::Point2f, cv::Point2f);
	bool intersect(const Line&, cv::Point2f&) const;
	cv::Point2f a() const { return _a; }
	cv::Point2f b() const { return _b; }
 private:
	cv::Point2f _a;
	cv::Point2f _b;
};

#endif  // _LINE_H_
