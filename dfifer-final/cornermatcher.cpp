#include "cornermatcher.h"

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#ifndef max
#define max(a,b)	((a)>(b)?(a):(b))
#endif

CornerMatcher::CornerMatcher(const Chessboard& cb1, const Chessboard& cb2)
		: _cb1(cb1), _cb2(cb2) {}


// Based on code from
// http://docs.opencv.org/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html
void
CornerMatcher::match()
{
	cv::Mat pat = _cb1.getSrcImg();
	cv::Mat g1, g2;
	cv::Mat t1, t2;

	cv::cvtColor(_cb1.getSrcImg(), t2, CV_BGR2GRAY);
	cv::cvtColor(_cb2.getSrcImg(), t1, CV_BGR2GRAY);

	cv::blur(t1, g1, cv::Size(5,5));
	cv::blur(t2, g2, cv::Size(5,5));

	cv::SurfFeatureDetector detector(200);
	std::vector<cv::KeyPoint> key1, key2;

	detector.detect(g1, key1);
	detector.detect(g2, key2);

	cv::SurfDescriptorExtractor extractor;
	//cv::OrbDescriptorExtractor extractor;
	cv::Mat desc1, desc2;

	extractor.compute(g1, key1, desc1);
	extractor.compute(g2, key2, desc2);

	cv::FlannBasedMatcher matcher;
//	cv::BFMatcher matcher(cv::NORM_L2, true);
	std::vector<cv::DMatch> m1, m;
	matcher.match(desc1, desc2, m1);

	double min_dist = 100, max_dist = 0;

	for (int i = 0; i < desc1.rows; ++i) {
		double d = m1[i].distance;
		if (d < min_dist) min_dist = d;
		if (d > max_dist) max_dist = d;	
	}

	for( int i = 0; i < desc1.rows; i++ ){
		if(m1[i].distance <= 2*min_dist) {
			m.push_back(m1[i]);
		}
	}
	
	std::cout << "Found " << m.size() << " good matches\n";

	cv::Mat img;
	cv::drawMatches(g1, key1, g2, key2, m, img, 
	//cv::drawMatches(g2, key2, g1, key1, m, img, 
			cv::Scalar::all(-1), cv::Scalar::all(-1),
			std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	
	cv::imwrite("matches.jpg", img);
	
	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;

	for( int i = 0; i < m.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		scene.push_back( key2[ m[i].trainIdx ].pt );
		obj.push_back( key1[ m[i].queryIdx ].pt );
	}

	cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC, 5);

	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point2f(0,0); obj_corners[1] = cv::Point2f( pat.cols, 0 );
	obj_corners[2] = cv::Point2f( pat.cols, pat.rows ); obj_corners[3] = cv::Point2f( 0, pat.rows );
	std::vector<cv::Point2f> scene_corners(4);

  cv::perspectiveTransform( obj_corners, scene_corners, H);

	cv::Point2f offset (pat.cols, 0);
	cv::line( img, scene_corners[0] + offset, scene_corners[1] + offset, cv::Scalar(0,255,0), 4);
	cv::line( img, scene_corners[1] + offset, scene_corners[2] + offset, cv::Scalar(0,255,0), 4);
	cv::line( img, scene_corners[2] + offset, scene_corners[3] + offset, cv::Scalar(0,255,0), 4);
	cv::line( img, scene_corners[3] + offset, scene_corners[0] + offset, cv::Scalar(0,255,0), 4);
	
	std::cout << "Pattern corners\n";
	for (auto it = obj_corners.begin(); it != obj_corners.end(); ++it)
		std::cout << *it << std::endl;

	std::cout << "Object corners\n";
	for (auto it = scene_corners.begin(); it != scene_corners.end(); ++it)
		std::cout << *it << std::endl;

	cv::imwrite("matches.jpg", img);
}
