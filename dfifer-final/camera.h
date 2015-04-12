#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <opencv2/opencv.hpp>

typedef struct CameraParams {
	// Pixel Size
	double dx, dy;
	// Focal Length
	double f;
	// Image Center
	double Cx, Cy;
	// Skew
	double k;
	// Rotation
	double r1, r2, r3, r4, r5, r6, r7, r8, r9;
	// Translation
	double Tx, Ty, Tz;
	// Calibration points
	std::vector<cv::Point3f> worldPts;
	std::vector<cv::Point2f> imgPts;
} CameraParams;

class Camera {
 public:
	Camera();
	void setInternals(float f, float dx, float dy);
	// Calibrate using the provided image
	bool calibrate(cv::Mat& m);
	bool cvCalibrate(cv::Mat& m);
		
 private:
	CameraParams _cameraParams;
	cv::Mat _rMat;
	cv::Mat _tVec;
	cv::Mat _camMat;
	cv::Mat _distCoeffs;

};

#endif  // _CAMERA_H_
