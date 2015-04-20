#include "feature_matcher.h"

#include <opencv2/nonfree/features2d.hpp>
#include <vector>

FeatureMatcher::FeatureMatcher(const cv::Mat& pat, const cv::Mat& img)
	: _pat(pat), _img(img) {}

cv::Mat
FeatureMatcher::match()
{
	cv::SurfFeatureDetector detector(400);
	std::vector<cv::KeyPoint> key1, key2;

	detector.detect(_pat, key1);
	detector.detect(_img, key2);

	cv::SurfDescriptorExtractor extractor;

	cv::Mat desc1, desc2;
	extractor.compute(_pat, key1, desc1);
	extractor.compute(_img, key2, desc2);

	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(desc1, desc2, matches);

	double max_dist = 0; double min_dist = 100;

	for (int i = 0; i < desc1.rows; i++) { 
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	std::vector<cv::DMatch> good_matches;

	for (int i = 0; i < desc1.rows; ++i) {
//		if (matches[i].distance < 3*min_dist) {
			good_matches.push_back(matches[i]);
//		}
	}

	cv::Mat img_matches;
	cv::drawMatches(_pat, key1, _img, key2, good_matches, img_matches,
			cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
			cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;

	for( int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( key1[ good_matches[i].queryIdx ].pt );
		scene.push_back( key2[ good_matches[i].trainIdx ].pt );
	}

	cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );
	cv::Mat H1 = cv::findHomography( scene, obj, CV_RANSAC );

	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point2f(0,0); 
	obj_corners[1] = cv::Point2f( _pat.cols, 0 );
	obj_corners[2] = cv::Point2f( _pat.cols, _pat.rows );
	obj_corners[3] = cv::Point2f( 0, _pat.rows );
	std::vector<cv::Point2f> scene_corners(4);

	cv::perspectiveTransform( obj_corners, scene_corners, H);

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	cv::Point2f offset(_pat.cols, 0);
	cv::line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, cv::Scalar(0, 255, 0), 4 );
	cv::line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, cv::Scalar(0, 255, 0), 4 );
	cv::line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, cv::Scalar(0, 255, 0), 4 );
	cv::line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, cv::Scalar(0, 255, 0), 4 );

	//-- Show detected matches

	cv::imwrite("matches.jpg", img_matches);

	cv::Mat warp;
	cv::warpPerspective(_img, warp, H1, _pat.size());
	cv::imwrite("recovered.jpg", warp);

	return warp;

}
