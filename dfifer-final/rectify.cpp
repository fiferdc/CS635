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


	cv::Mat gray, hsv, lab;
	cv::cvtColor(input, gray, CV_BGR2GRAY);
	cv::imwrite("gray.jpg", gray);
		
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
	
	cv::Size dsize((_grid.width-1)*100, (_grid.height-1)*100);
	cv::Mat r(dsize, input.type());

	cv::warpPerspective(input, r, t, dsize);
	out = r;
	_transformation = t;

	cv::cvtColor(r, lab, CV_BGR2Lab);
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

//	std::cout << "CENTERS\n";
//	std::cout << centers << std::endl;


	

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
	for (int x = 1; x < _grid.width-1; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			for (int z = -3; z <= 3; ++z) {
				canny.at<uchar>(y,x*100+z) = 0;
			}
		}
	}
	
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 1; y < _grid.height-1; ++y) {
			for (int z = -3; z <= 3; ++z) {
				canny.at<uchar>(y*100+z,x) = 0;
			}
		}
	}	
	cv::imwrite("canny.jpg", canny);

	// Detect lines
	std::vector<cv::Vec2f> lines;
	cv::Mat dlines = clusters.clone();
	cv::HoughLines(canny, lines, 3, CV_PI/180, 150, 0, 0);

	// From http://opencvexamples.blogspot.com/2013/10/line-detection-by-hough-line-transform.html
	for( size_t i = 0; i < lines.size(); i++ )
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line( dlines, pt1, pt2, cv::Scalar(255,255,255), 1, CV_AA);
	}

	cv::imwrite("detected.jpg", dlines);

	return true;

}
