#include "feature_matcher.h"

#include <opencv2/nonfree/features2d.hpp>
#include <vector>

using namespace cv;

FeatureMatcher::FeatureMatcher(const Mat& pat, const Mat& img)
	: _pat(pat), _img(img) {}

Mat
FeatureMatcher::match()
{
	SurfFeatureDetector detector(500);
	std::vector<KeyPoint> key1, key2;

	detector.detect(_pat, key1);
	detector.detect(_img, key2);

	SurfDescriptorExtractor extractor;

	Mat desc1, desc2;
	extractor.compute(_pat, key1, desc1);
	extractor.compute(_img, key2, desc2);

	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match(desc1, desc2, matches);

	double max_dist = 0; double min_dist = 100;

	for (int i = 0; i < desc1.rows; i++) { 
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	std::vector<DMatch> good_matches;

	for (int i = 0; i < desc1.rows; ++i) {
		if (matches[i].distance < 3*min_dist) {
			good_matches.push_back(matches[i]);
		}
	}

	Mat img_matches;
	drawMatches(_pat, key1, _img, key2, good_matches, img_matches,
			Scalar::all(-1), Scalar::all(-1), std::vector<char>(),
			DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for( int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( key1[ good_matches[i].queryIdx ].pt );
		scene.push_back( key2[ good_matches[i].trainIdx ].pt );
	}

	double ransac_thresh = 3;
	Mat H = findHomography( obj, scene, CV_RANSAC, ransac_thresh );
	Mat H1 = findHomography( scene, obj, CV_RANSAC, ransac_thresh );

	_patToImg = H.clone();
	_imgToPat = H1.clone();

	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = Point2f(0,0); 
	obj_corners[1] = Point2f( _pat.cols, 0 );
	obj_corners[2] = Point2f( _pat.cols, _pat.rows );
	obj_corners[3] = Point2f( 0, _pat.rows );
	std::vector<Point2f> scene_corners(4);

	perspectiveTransform( obj_corners, scene_corners, H);

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	Point2f offset(_pat.cols, 0);
	line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar(0, 255, 0), 4 );
	line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar(0, 255, 0), 4 );

	//-- Show detected matches

	imwrite("matches.jpg", img_matches);

	Mat warp;
	warpPerspective(_img, warp, H1, _pat.size());
	imwrite("recovered.jpg", warp);

	_patFeat = key1;
	_imgFeat = key2;
	_matches = good_matches;

	return warp;
}

std::vector<Point2f>
FeatureMatcher::getPatFeatures()
{
	std::vector<Point2f> pts;
	for (auto m = _matches.begin(); m != _matches.end(); ++m) {
		pts.push_back(_patFeat[m->queryIdx].pt);
	}
	return pts;
}

std::vector<Point2f>
FeatureMatcher::getImgFeatures()
{
	std::vector<Point2f> pts;
	for (auto m = _matches.begin(); m != _matches.end(); ++m) {
		pts.push_back(_patFeat[m->trainIdx].pt);
	}
	return pts;
}

