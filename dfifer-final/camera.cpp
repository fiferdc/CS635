#include "camera.h"

#include <opencv2/opencv.hpp>

Camera::Camera(int width, int height) {
  _cameraParams.f = 0.0;
	_width = width;
	_height = height;
	_calibrated = false;
}
  
void
Camera::setInternals(float f, float dx, float dy)
{
  _cameraParams.f = f;
  _cameraParams.dx = dx;
  _cameraParams.dy = dy;
}

#define SIGN(X) ((X)<0?-1:1)

#define CVMAT(M,i,j) (M).at<double>((i),(j))

//#define DEBUG_PRINT

// Global camera params for use with lmdif
CameraParams *cpPtr;

typedef void (*lmdif_fcn)(int*,int*,double*,double*,int*);
extern "C" {
	void lmdif_(lmdif_fcn, int*, int*, double*, double*, double*, double*, 
							double*, int*, double*, double*, int*, double*, int*, int*, int*, 
							double*, int*, int*, double*, double*, double*, double*, double*);
}

///////////////////////////////////////////////////////////////////////////////
// Coplanar Calibration
///////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------
 Solves r1/Ty, r2/Ty, Tx/Ty, r4/Ty, r5/Ty
 ----------------------------------------------------------------------------*/

void 
fcn1(int *m, int *n, double* x, double* fvec, int *iflag)
{
	static int count = 0;
	static int printMask = 1;
	for (int i = 0; i < *m; ++i) {
		double Xd = cpPtr->dx * (cpPtr->imgPts[i].x - cpPtr->Cx);
		double Yd = cpPtr->dy * (cpPtr->imgPts[i].y - cpPtr->Cy);
		double xw = cpPtr->worldPts[i].x;
		double yw = cpPtr->worldPts[i].y;
		double err = Yd*xw*x[0] + Yd*yw*x[1] + Yd*x[2] -Xd*xw*x[3] - Xd*yw*x[4]
				- Xd;
		fvec[i] = err*err;
	}
	count++;
	if (count % printMask == 0) {
		if (count == printMask * 10) printMask *= 10;
		double err = 0.0;
		for (int i = 0; i < *m; ++i)
			err += sqrt(fvec[i]);
		//printf("%4i: ");
		//for (int i = 0; i < *n; ++i)
		//	printf("x%i=%lf ", i, x[i]);
		//printf("err=%lf\n", count, x[0], x[1], err/(double)*m); 
	}
}

/*-----------------------------------------------------------------------------
 Solves f, Tz
 ----------------------------------------------------------------------------*/

void
fcn2(int* m, int* n, double* x, double* fvec, int* iflag)
{
	static int count = 0;
	static int printMask = 1;
	for (int i = 0; i < *m; ++i) {
		double Yi = cpPtr->imgPts[i].y - cpPtr->Cy;
		double xw = cpPtr->worldPts[i].x;
		double yw = cpPtr->worldPts[i].y;
		double yi = cpPtr->r4*xw + cpPtr->r5*yw + cpPtr->Ty;
		double wi = cpPtr->r7*xw + cpPtr->r8*yw;
		double err = yi*x[0] - cpPtr->dy*Yi*x[1] - wi*cpPtr->dy*Yi;
		fvec[i] = err*err;
	}
	count++;
	if (count % printMask == 0) {
		if (count == printMask * 10) printMask *= 10;
		double err = 0.0;
		for (int i = 0; i < *m; ++i)
			err += sqrt(fvec[i]);
		//printf("%4i: f=%.8lf Tz=%.8lf err=%lf\n", count, x[0], x[1], 
		//		err/(double)*m); 
	}
}

/*-----------------------------------------------------------------------------
 Optimizes f, Cx, Cy
 ----------------------------------------------------------------------------*/

void
optIn(int* m, int* n, double* x, double* fvec, int* iflag)
{
	double f = x[0];
	double Cx = x[1], Cy = x[2];
	double dx = cpPtr->dx, dy = cpPtr->dy;
	double Tx = x[3], Ty = x[4], Tz = x[5];
//	double Tx = cpPtr->Tx, Ty = cpPtr->Ty, Tz = cpPtr->Tz;
	double r1 = cpPtr->r1, r2 = cpPtr->r2, r3 = cpPtr->r3,
				 r4 = cpPtr->r4, r5 = cpPtr->r5, r6 = cpPtr->r6,
				 r7 = cpPtr->r7, r8 = cpPtr->r8, r9 = cpPtr->r9;

	for (int i = 0; i < *m; ++i) {
		double X = cpPtr->worldPts[i].x;
		double Y = cpPtr->worldPts[i].y;
		double Z = cpPtr->worldPts[i].z;
		double Xf = cpPtr->imgPts[i].x;
		double Yf = cpPtr->imgPts[i].y;
		double x = r1*X + r2*Y + r3*Z + Tx;
		double y = r4*X + r5*Y + r6*Z + Ty;
		double z = r7*X + r8*Y + r9*Z + Tz;
		double Xu = f * x / z;
		double Yu = f * y / z;
		double Xf_ = Xu / dx + Cx;
		double Yf_ = Yu / dy + Cy;
		double dx = Xf - Xf_;
		double dy = Yf - Yf_;
		fvec[i] = dx*dx + dy*dy;
	}

}


/*----------------------------------------------------------------------------*/

bool 
Camera::calibrate(cv::Mat& input)
{
	
	// Find corners
	cv::Size patternSize(6, 6);
	std::vector<cv::Point2f> corners;

	bool found = cv::findChessboardCorners(input, patternSize, corners);
	
	if (!found) {
		printf("ERROR: Find corners failed\n");
		return false;
	}
	
	cv::Mat gray;
	cv::cvtColor(input, gray, CV_BGR2GRAY);

	cv::cornerSubPix(gray, corners, cv::Size(6,6), cv::Size(-1,-1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	
	cv::drawChessboardCorners(input, patternSize, cv::Mat(corners), found);

	cv::imwrite("found.jpg", input);

  _cameraParams.worldPts.clear();
	
	cv::Point2f a = corners[1] - corners[0];
	cv::Point2f b = corners[6] - corners[0];
	// Look at sign of z in the cross product for the corner orientation
	if (a.x*b.y - a.y*b.x < 0) {
  	for (int i = 0; i < 6; ++i) {
			for (int j = 0; j < 6; ++j) {
				cv::Point3f p(25.4*j, -25.4*i, 1.0);
				std::cout << p;
				_cameraParams.worldPts.push_back(p);
			}
		}
	} else {
		for (int j = 0; j < 6; ++j) {
			for (int i = 0; i < 6; ++i) {
				cv::Point3f p(25.4*(5-j), -25.4*i, 1.0);
				std::cout << p;
				_cameraParams.worldPts.push_back(p);
			}
		}
	
	}

  _cameraParams.imgPts.clear();
	for (int i = 0; i < 36; ++i) {
		_cameraParams.imgPts.push_back(corners[i]);
		std::cout << corners[i] << std::endl;
	}

  cpPtr = &_cameraParams;
  cv::Size imgSize = input.size();
  _cameraParams.Cx = imgSize.width / 2;
  _cameraParams.Cy = imgSize.height / 2;

	int m, n, maxfev, mode, nprint, info, nfev, ldfjac;
	int *ipvt;
	double ftol, xtol, gtol, epsfcn, factor;
	double *x1, *fvec, *diag, *fjac, *qtf, *wa1, *wa2, *wa3, *wa4;
	
	m = _cameraParams.imgPts.size();
	n = 5;
	x1 = new double[n];
	for (int i = 0; i < n; ++i)
		x1[i] = 0.0;
	fvec = new double[m];
	ftol = 1.0e-7;
	xtol = 1.0e-7;
	gtol = 0.0;
	maxfev = 1000*n;
	epsfcn = 1.0e-16;
	diag = new double[n];
	mode = 1;
	factor = 10;
	nprint = 1;
	fjac = new double[m*n];
	ldfjac = m;
	ipvt = new int[n];
	qtf = new double[n];
	wa1 = new double[n];	
	wa2 = new double[n];	
	wa3 = new double[n];
	wa4 = new double[m];	

	// change to coplanar function
	// Compute r1/Ty, r2/Ty, Tx/Ty, r4/Ty, r5/Ty = x[0]...x[4]
	lmdif_(fcn1, &m, &n, x1, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn, diag, 
				 &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac, ipvt, qtf, wa1, 
				 wa2, wa3, wa4);

#ifdef DEBUG_PRINT
	printf("Done solving for r1/Ty, r2/Ty, Tx/Ty, r4/Ty, r5/T1\n");
	printf("\tInfo: %i\n", info);
	printf("\tNumber of iterations: %i\n", nfev);
	double err = 0.0;
	for (int i = 0; i < m; ++i)
		err += fvec[i];
	err /= m;
	printf("\tPer-Pixel error: %lf\n", sqrt(err));
#endif

	cv::Mat A1(m,5,CV_64F);
	cv::Mat b1(m,1,CV_64F);
	cv::Mat m1(5,1,CV_64F);
	for (int i = 0; i < m; ++i) {
		double Xd = cpPtr->dx * (cpPtr->imgPts[i].x - cpPtr->Cx);
		double Yd = cpPtr->dy * (cpPtr->imgPts[i].y - cpPtr->Cy);
		double xw = cpPtr->worldPts[i].x;
		double yw = cpPtr->worldPts[i].y;
		CVMAT(A1,i,0) = Yd*xw;
		CVMAT(A1,i,1) = Yd*yw;
		CVMAT(A1,i,2) = Yd;
		CVMAT(A1,i,3) = -Xd*xw;
		CVMAT(A1,i,4) = -Xd*yw;
		CVMAT(b1,i,0) = Xd;
	}
	cv::solve(A1,b1,m1,cv::DECOMP_SVD);
	for (int i = 0; i < 5; ++i) {
#ifdef DEBUG_PRINT
		printf("%lf << %lf\n", x1[i], CVMAT(m1,i,0));
#endif
		x1[i] = CVMAT(m1,i,0);
	}

	// Compute |Ty|
	double r1_ = x1[0], r2_ = x1[1], r4_ = x1[3], r5_ = x1[4];
	double detr = r1_*r5_ - r4_*r2_;
	double Ty2;
	if (detr != 0) {
		double Sr = r1_*r1_ + r2_*r2_ + r4_*r4_ + r5_*r5_;
		Ty2 = (Sr - sqrt(Sr*Sr - 4*detr*detr))/(2*detr*detr);
	} else {
		double denom = 0.0;
		if (r1_ != 0) denom += r1_*r1_;
		if (r2_ != 0) denom += r2_*r2_;
		if (r4_ != 0) denom += r4_*r4_;
		if (r5_ != 0) denom += r5_*r5_;
		Ty2 = 1.0 / denom;
	}
	double Ty = sqrt(Ty2);
	
	// Determine sign of Ty
	// Grab a point far from the image center
	int i = 0;
	double max_distance = 0.0;
	for (int j = 0; j < _cameraParams.imgPts.size(); ++j) {
		cv::Point2f delta = _cameraParams.imgPts[j] - cv::Point2f(_cameraParams.Cx, _cameraParams.Cy);
		double distance = abs(delta.x) + abs(delta.y);
		if (distance > max_distance) {
			i = j;
			max_distance = distance;
		}	
	}

	double X = _cameraParams.imgPts[i].x - _cameraParams.Cx;
	double Y = _cameraParams.imgPts[i].y - _cameraParams.Cy;
	double xw = _cameraParams.worldPts[i].x;
	double yw = _cameraParams.worldPts[i].y;

	int sgn = 1;
	double r1 = r1_ * Ty;
	double r2 = r2_ * Ty;
	double r4 = r4_ * Ty;
	double r5 = r5_ * Ty;
	double Tx = x1[2] * Ty;
	double x = r1*xw + r2*yw + Tx;
	double y = r4*xw + r5*yw + Ty;
	sgn = (SIGN(x) == SIGN(X) && SIGN(y) == SIGN(Y))?1:-1;
	Ty *= sgn;

	r1 *= sgn;
	r2 *= sgn;
	Tx *= sgn;
	r4 *= sgn;
	r5 *= sgn;

	_cameraParams.r1 = r1;
	_cameraParams.r2 = r2;
	_cameraParams.r4 = r4;
	_cameraParams.r5 = r5;
	_cameraParams.Tx = Tx;
	_cameraParams.Ty = Ty;

	// Compute 3D rotation matrix R
	double r3 = sqrt(1.0 - r1*r1 - r2*r2);
	double s = -SIGN(r1*r4 + r2*r5);
	double r6 = s*sqrt(1.0 - r4*r4 - r5*r5);
	cv::Point3f row1(r1, r2, r3);
	cv::Point3f row2(r4, r5, r6);
	cv::Point3f row3 = row2.cross(row1);
	double r7 = row3.x;
	double r8 = row3.y;
	double r9 = row3.z;

	_cameraParams.r3 = r3;
	_cameraParams.r6 = r6;
	_cameraParams.r7 = r7;
	_cameraParams.r8 = r8;
	_cameraParams.r9 = r9;

	// Compute an approximation of f and Tz
	n = 2;
	double *x2 = new double[n];
	
	// Solve f, Tz linearly	
	cv::Mat A2(m,2,CV_64F);
	cv::Mat b2(m,1,CV_64F);
	cv::Mat m2(2,1,CV_64F);
	for (int i = 0; i < m; ++i) {
		double xw = _cameraParams.worldPts[i].x;
		double yw = _cameraParams.worldPts[i].y;
		double yi = r4*xw + r5*yw + Ty;
		double Yi = _cameraParams.imgPts[i].y - _cameraParams.Cy;
		double wi = r7*xw + r8*yw;
		CVMAT(A2,i,0) = yi;
		CVMAT(A2,i,1) = -_cameraParams.dy*Yi;
		CVMAT(b2,i,0) = wi*_cameraParams.dy*Yi;
	}
	cv::solve(A2,b2,m2,cv::DECOMP_QR);
	for (int i = 0; i < 2; ++i) {
		x2[i] = CVMAT(m2,i,0);
#ifdef DEBUG_PRINT
		printf("x2[%i] = %lf\n", i, x2[i]);
#endif
	}

	// Optimize f, Tz
	maxfev = 1000*n;
	free(diag);	diag = new double[n];
	free(fjac);	fjac = new double[m*n];
	free(ipvt);	ipvt = new int[n];
	free(qtf);	qtf = new double[n];
	free(wa1);	wa1 = new double[n];	
	free(wa2);	wa2 = new double[n];	
	free(wa3);	wa3 = new double[n];
	for (int i = 0; i < m; ++i)
		fvec[i] = 0.0;
	
	lmdif_(fcn2, &m, &n, x2, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn, diag, 
				 &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac, ipvt, qtf, wa1, 
				 wa2, wa3, wa4);

#if defined(DEBUG_PRINT)	
	for (int i = 0; i < 2; ++i) {
		printf("x2[%i] = %lf\n", i, x2[i]);
	}

	printf("Done solving for f and Tz\n");
	printf("\tInfo: %i\n", info);
	printf("\tNumber of iterations: %i\n", nfev);
	err = 0.0;
	for (int i = 0; i < m; ++i)
		err += fvec[i];
	err /= m;
	printf("\tPer-Pixel error: %lf\n", sqrt(err));
#endif

	double f = x2[0];
	double Tz = x2[1];
	
	//f = cp.f;
	_cameraParams.f = f;
	_cameraParams.Tz = Tz;

	// Assign values to R based on f
	if (f < 0) {
		r3 = -r3;
		r6 = -r6;
		r7 = -r7;
		r8 = -r8;
		_cameraParams.r3 = r3;
		_cameraParams.r6 = r6;
		_cameraParams.r7 = r7;
		_cameraParams.r8 = r8;
		f = -f;
		Tz = -Tz;
		_cameraParams.f = f;
		_cameraParams.Tz = Tz;
	}

	double theta = -asin(r3);
	double phi = atan2(r6/cos(theta), r9/cos(theta));
	double psi = atan2(r2/cos(theta), r1/cos(theta));	
	
	// Optimize all parameters
	
	n = 4;
	double x3[] = { f, _cameraParams.Cx, _cameraParams.Cy, Tx, Ty, Tz }; //theta, phi, psi, Tx, Ty, Tz };
	maxfev = 1000*n;
	free(diag);	diag = new double[n];
	free(fjac);	fjac = new double[m*n];
	free(ipvt);	ipvt = new int[n];
	free(qtf);	qtf = new double[n];
	free(wa1);	wa1 = new double[n];	
	free(wa2);	wa2 = new double[n];	
	free(wa3);	wa3 = new double[n];
	for (int i = 0; i < m; ++i)
		fvec[i] = 0.0;
	
	lmdif_(optIn, &m, &n, x3, fvec, &ftol, &xtol, &gtol, &maxfev, &epsfcn, diag, 
				 &mode, &factor, &nprint, &info, &nfev, fjac, &ldfjac, ipvt, qtf, wa1, 
				 wa2, wa3, wa4);

	f = _cameraParams.f = x3[0];
	_cameraParams.Cx = x3[1];
	_cameraParams.Cy = x3[2];
	Tx = _cameraParams.Tx = x3[3];
	Ty = _cameraParams.Ty = x3[4];
	Tz = _cameraParams.Tz = x3[5];
/*	{
		double ct = cos(theta), cf = cos(phi), cp = cos(psi);
		double st = sin(theta), sf = sin(phi), sp = sin(psi);
		r1 = cp*ct;
		r2 = sp*ct;
		r3 = -st;	
		r4 = -sp*cf + cp*st*cf;
		r5 = cp*cf + sp*st*sf;
		r6 = ct*sf;
		r7 = sp*sf + cp*st*cf;
		r8 = -cp*sf + sp*st*cf;
		r9 = ct*cf;
	}
	cp.r1 = r1;
	cp.r2 = r2;
	cp.r3 = r3;
	cp.r4 = r4;
	cp.r5 = r5;
	cp.r6 = r6;
	cp.r7 = r7;
	cp.r8 = r8;
	cp.r9 = r9;
*/

  double r[9] = {r1, r2, r3, r4, r5, r6, r7, r8, r9};
  _rMat = cv::Mat(3, 3, CV_64F, r);
	cv::transpose(_rMat, _rMatT);
	double k[9] = {f/_cameraParams.dx, 0.0, _cameraParams.Cx, 0.0, f/_cameraParams.dy, _cameraParams.Cy, 0.0, 0.0, 1.0};
  _camMat = cv::Mat(3, 3, CV_64F, k);
	double t[3] = {Tx, Ty, Tz};
  _tVec = cv::Mat(3, 1, CV_64F, t);

	printf("\nCalibration Complete!\n");
	printf("------------------------\n");
	printf("Intrinsic Parameters:\n");
	printf("f: %lf\n", _cameraParams.f);
	printf("Cx: %lf\n", _cameraParams.Cx);
	printf("Cy: %lf\n", _cameraParams.Cy);
	printf("k: 0\n");
	//K.print(stdout);
	printf("\n");
	printf("Extrinsic Parameters:\n");
	printf("R:\n");
	std::cout << _rMat << std::endl;
	printf("T:\n");
	std::cout << _tVec << std::endl;

//	calibrated = true;
  return true;
}

void 
Camera::cvCalibrate()
{
	
	if (_worldPts.size() == 0 || _imgPts.size() == 0 || _worldPts.size() != _imgPts.size()) {
		std::cerr << "ERROR: Invalid number of input points to calibration" << std::endl;
		exit(1);
	}
	
	std::vector<std::vector<cv::Point3f> > objectPoints (1);
	std::vector<std::vector<cv::Point2f> > imagePoints (1);
	objectPoints[0] = _worldPts;
	imagePoints[0] = _imgPts;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Size imageSize = cv::Size(_width, _height);

	std::cout << cameraMatrix << std::endl;

	cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs, 0);

	_rMat = rvecs[0];
	_tVec = tvecs[0];
	_camMat = cameraMatrix;
	_distCoeffs = distCoeffs;

	std::cout << cameraMatrix << std::endl;
	
	cv::Mat R_;
	cv::Rodrigues(rvecs[0], R_);
	std::cout << R_ << std::endl;
	std::cout << tvecs[0] << std::endl;

	_calibrated = true;
}

void
Camera::reset()
{
	_worldPts.clear();
	_imgPts.clear();
	_calibrated = false;
}

void
Camera::addPoint(cv::Point3f world, cv::Point2f img)
{
	_worldPts.push_back(world);
	_imgPts.push_back(img);
}

cv::Vec3f
Camera::camX() const
{
	float x = _rMat.at<float>(0,0);
	float y = _rMat.at<float>(0,1);
	float z = _rMat.at<float>(0,2);
	return cv::Vec3f(x, y, z);
}

cv::Vec3f
Camera::camY() const
{
	float x = _rMat.at<float>(1,0);
	float y = _rMat.at<float>(1,1);
	float z = _rMat.at<float>(1,2);
	return cv::Vec3f(x, y, z);
}

cv::Vec3f
Camera::camZ() const
{
	float x = _rMat.at<float>(2,0);
	float y = _rMat.at<float>(2,1);
	float z = _rMat.at<float>(2,2);
	return cv::Vec3f(x, y, z);
}

cv::Point3f
Camera::cop() const
{
	cv::Mat v(3,1, CV_32F);
	v.at<float>(0,0) = -_tVec.at<float>(0,0);
	v.at<float>(1,0) = -_tVec.at<float>(1,0);
	v.at<float>(2,0) = -_tVec.at<float>(2,0);
	cv::Mat w = _rMatT * v;
	float x = w.at<float>(0,0);
	float y = w.at<float>(1,0);
	float z = w.at<float>(2,0);
	return cv::Point3f(x, y, z);
}

//cv::Point3f reprojectToImage(cv::Point2f);
//cv::Point3f reproject(cv::Point2f, const Camera&, cv::Point2f);

cv::Point2f
Camera::project(cv::Point3f p) const
{
	std::vector<cv::Point3f> wPts(1);
	wPts[0] = p;
	std::vector<cv::Point2f> iPts;
	cv::projectPoints(wPts, _rMat, _tVec, _camMat, _distCoeffs, iPts);
	return iPts[0];
}
