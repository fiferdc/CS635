#include "rectify.h"

#include "feature_matcher.h"

using namespace cv;

Rectification::Rectification(const Mat& pat, const Mat& img)
		: _pat(pat), _img(img) {}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(a,l,h) MIN(MAX(a,l),h)


cv::Mat
Rectification::rectify()
{
	Mat img, img2, img3;

	Mat grayPat, grayImg;
	cvtColor(_pat, grayPat, CV_BGR2GRAY);
	cvtColor(_img, grayImg, CV_BGR2GRAY);
	
	// Perform initial rectification
	FeatureMatcher fm(grayPat, grayImg);
	fm.match();
	warpPerspective(grayImg, img, fm.getImgToPat(), _pat.size());
	
	// Rectify again for better match
	FeatureMatcher fm2(grayPat, img);
	fm2.match();

	_imgPts = fm.getImgFeatures();
	std::vector<Point2f> temp;
	perspectiveTransform(_imgPts, temp, fm.getImgToPat());
	perspectiveTransform(temp, _patPts, fm2.getImgToPat());
		
	_hPatToPat = fm2.getPatToImg();
	_hPatToImg = fm.getPatToImg();

	warpPerspective(_img, img2, fm.getImgToPat(), _pat.size());
	imwrite("img2.jpg", img2);
	
	_hImgToPat1 = fm.getImgToPat();
	_hImgToPat2 = fm2.getImgToPat();

	warpPerspective(_img, img2, fm.getImgToPat(), img2.size());
	warpPerspective(img2, img3, fm2.getImgToPat(), _pat.size());
	imwrite("img3.jpg", img3); 

	return img3;
}

cv::Mat 
Rectification::applyRectification(const cv::Mat& m) {
	cv::Mat img2, img3;
	warpPerspective(m, img2, _hImgToPat1, _pat.size());
	warpPerspective(img2, img3, _hImgToPat2, _pat.size());
	return img3;
}

