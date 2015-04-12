#include "line.h"

#include <opencv2/opencv.hpp>
#include <vector>

bool
FindLines(const cv::Mat& in, std::vector<Line>& out)
{
	cv::Mat lab;

	cv::Size dsize(700,700);
	cv::Size _grid(8,8);

	cv::cvtColor(in, lab, CV_BGR2Lab);
	cv::imwrite("lab.jpg", lab);

	std::vector<cv::Mat> Lab;
	cv::split(lab, Lab);
	
	Lab[0] = cv::Mat::zeros(Lab[0].size(), Lab[0].type());

	for (int y = 0; y < dsize.height; ++y) {
		for (int x = 0; x < dsize.width; ++x) {
			if ((x/100)%2 == (y/100)%2) {
				Lab[1].at<uchar>(y,x)=0;
				Lab[2].at<uchar>(y,x)=0;
			}
		}
	}

	// Verification
	cv::merge(&Lab[0], 3, lab);
	cv::imwrite("ab.jpg", lab);
	cv::imwrite("a.jpg", Lab[1]);
	cv::imwrite("b.jpg", Lab[2]);

	cv::Mat ab;
	cv::merge(&Lab[1], 2, ab);

	cv::Mat samples(dsize.width*dsize.height, 3, CV_32F);
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			samples.at<float>(y + x*dsize.height, 0) = 0;
			for (int z = 0; z < 2; ++z) {
				samples.at<float>(y + x*dsize.height, z+1) = ab.at<cv::Vec2b>(y,x)[z];
			}
		}
	}
	cv::Mat labels, centers;

	cv::kmeans(samples, 7, labels, 
			cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.001),
			3, cv::KMEANS_PP_CENTERS, centers);

	cv::Mat	clusters(dsize, CV_8UC3);
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			int label = labels.at<int>(y+dsize.height*x,0);
			clusters.at<cv::Vec3b>(y,x)[0] = centers.at<float>(label, 0);
			clusters.at<cv::Vec3b>(y,x)[1] = centers.at<float>(label, 1);
			clusters.at<cv::Vec3b>(y,x)[2] = centers.at<float>(label, 2);
		}
	}

//	cv::cvtColor(clusters, clusters, CV_Lab2BGR);
	cv::imwrite("clusters.jpg", clusters);
	
	// Perform Canny edge detector
	cv::Mat canny;
	cv::Canny(clusters, canny, 50, 200, 3);

	// Remove natural edges
	int erase = 4;
	for (int x = 1; x < _grid.width-1; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			for (int z = -erase; z <= erase; ++z) {
				canny.at<uchar>(y,x*100+z) = 0;
			}
		}
	}
	
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 1; y < _grid.height-1; ++y) {
			for (int z = -erase; z <= erase; ++z) {
				canny.at<uchar>(y*100+z,x) = 0;
			}
		}
	}	
	cv::imwrite("canny.jpg", canny);

	// Detect lines
	std::vector<cv::Vec2f> lines;
	cv::Mat dlines = clusters.clone();
	cv::HoughLines(canny, lines, 3, CV_PI/180, 50, 0, 0);

	// From http://opencvexamples.blogspot.com/2013/10/line-detection-by-hough-line-transform.html
	out.clear();
	std::cout << "Lines detected:\n";
	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		std::cout << "Rho: " << rho << "\tTheta: " << theta << std::endl;
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		out.push_back(Line(pt1, pt2));
		line( dlines, pt1, pt2, cv::Scalar(255,255,255), 1, CV_AA);
	}
	std::cout << std::endl;

	cv::imwrite("detected.jpg", dlines);

	return true;
}

void
FindIntersections(const std::vector<Line>& h, const std::vector<Line>& v, std::vector<cv::Point2f>& ret)
{
	ret.clear();
	for (std::vector<Line>::const_iterator it = h.cbegin(); it != h.cend(); it++) {
		for (std::vector<Line>::const_iterator it2 = v.cbegin(); it2 != v.cend(); it2++) {
			cv::Point2f i;
			if (it->intersect(*it2, i)) {
					ret.push_back(i);
			}
		}
	}
}
