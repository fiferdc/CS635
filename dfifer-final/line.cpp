#include "line.h"

/**
 * A line is defined geometrically by:
 *	a + t * (b - a)
 */

Line::Line(cv::Point2f a, cv::Point2f b)
		: _a(a), _b(b) {}

bool
Line::intersect(const Line& line, cv::Point2f& intersection) const
{

	cv::Point2f x = line.a() - _a;
	cv::Point2f d1 = _b - _a;
	cv::Point2f d2 = line.b() - line.a();

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < 1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x)/cross;
	intersection = _a + d1 * t1;
	return true;
}

