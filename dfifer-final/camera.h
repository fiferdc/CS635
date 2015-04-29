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
	Camera(int, int);
	void setInternals(float f, float dx, float dy);
	// Calibrate using the provided image
	bool calibrate(cv::Mat& m);
	void cvCalibrate();
	bool isCalibrated() { return _calibrated; }
	void reset();
	void addPoint(cv::Point3f, cv::Point2f);
	cv::Mat getCameraMatrix() { return _camMat; }
	cv::Vec3f camX() const;
	cv::Vec3f camY() const;
	cv::Vec3f camZ() const;
	cv::Point3f cop() const;
	cv::Point3f reprojectToImage(cv::Point2f) const;
	cv::Point3f reproject(cv::Point2f, const Camera&, cv::Point2f) const;
	cv::Point2f	project(cv::Point3f) const;
	void printError(const char*) const;
	cv::Mat getProjectionMat() const;
	cv::Point2f undistortPoint(cv::Point2f) const;
	cv::Point2f distortPoint(cv::Point2f) const;

 private:
	int _width;
	int _height;
	CameraParams _cameraParams;
	cv::Mat _rMat;	// Projection Rotation
	cv::Mat _rMatT;	// Inverse Rotation matrix for reprojection
	cv::Mat _tVec;
	cv::Mat _camMat;
	cv::Mat _distCoeffs;
	std::vector<cv::Point3f> _worldPts;
	std::vector<cv::Point2f> _imgPts;
	bool _calibrated;
};

#endif  // _CAMERA_H_
