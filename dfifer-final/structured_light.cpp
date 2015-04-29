#include "structured_light.h"

SLPair::SLPair(const cv::Mat& on, const cv::Mat& off)
		: _on(on), _off(off)
{
	_classification = cv::Mat(_on.size(), CV_32FC1);
	for (int x = 0; x < _on.cols; ++x) {
		for (int y = 0; y < _on.rows; ++y) {
			float f1 = _on.at<cv::Vec3b>(y,x)[0];
			float f2 = _off.at<cv::Vec3b>(y,x)[0];
			float f = -1.0;

			if (abs(f1-f2)/255.0 > 0.05) {
				f = (f1 > f2)?1.0:0.0;
			}
			_classification.at<float>(y,x) = f;
		}
	}	
}


int
SLPair::getClassification(int x, int y)
{
	return (int)_classification.at<float>(y,x);
}


StructuredLight::StructuredLight(cv::Size dim)
		: _dim(dim)
{
	_projId = cv::Mat::zeros(_dim, CV_32FC3);
	_centers = cv::Mat::zeros(cv::Size(600,600), CV_32FC3);
}


void 
StructuredLight::addImagePair(const cv::Mat& on, const cv::Mat& off, int a)
{
	if (a == 0)
		_h.push_back(SLPair(on, off));
	else
		_v.push_back(SLPair(on, off));
}

void
StructuredLight::decode()
{
	// X - Horizontal ID
	// Y - Vertical ID
	// Z - 0 if decoding is valid, 1 othterwise
	
	for (auto it = _h.begin(); it != _h.end(); ++it) {
		for (int x = 0; x < _dim.width; ++x) {
			for (int y = 0; y < _dim.height; ++y) {
				float f = _projId.at<cv::Vec3f>(y,x)[0];
				int c = it->getClassification(x,y);
				if (c == -1)
					_projId.at<cv::Vec3f>(y,x)[2] = 1;
				f = f * 2 + c;
				_projId.at<cv::Vec3f>(y,x)[0] = f;
			}
		}	
	}
	for (auto it = _v.begin(); it != _v.end(); ++it) {
		for (int x = 0; x < _dim.width; ++x) {
			for (int y = 0; y < _dim.height; ++y) {
				float f = _projId.at<cv::Vec3f>(y,x)[1];
				int c = it->getClassification(x,y);
				if (c == -1)
					_projId.at<cv::Vec3f>(y,x)[2] = 1;
				f = f * 2 + c;
				_projId.at<cv::Vec3f>(y,x)[1] = f;
			}
		}	
	}
	
	// Find centers
	for (int x = 0; x < _dim.width; ++x) {
		for (int y = 0; y < _dim.height; ++y) {
			cv::Vec3f v = _projId.at<cv::Vec3f>(y,x);
			if (v[2] == 0) {
				float X = v[0], Y = v[1];
				_centers.at<cv::Vec3f>(Y,X)[0] += x;
				_centers.at<cv::Vec3f>(Y,X)[1] += y;
				_centers.at<cv::Vec3f>(Y,X)[2]++;
			}
		}
	}

	for (int x = 0; x < 600; ++x) {
		for (int y = 0; y < 600; ++y) {
			float f =  _centers.at<cv::Vec3f>(y,x)[2];
			if (f > 0) {
				_centers.at<cv::Vec3f>(y,x)[0] /= f;
				_centers.at<cv::Vec3f>(y,x)[1] /= f;
			}
		}
	}

}

cv::Mat
StructuredLight::getColored()
{
	const float r[3] = { 0, 0, 255};
	const float g[3] = { 0, 255, 0};
	const float m[3] = { 0, 255, 255};
	const float b[3] = { 255, 0, 0};
	cv::Mat c(_dim, CV_8UC3);

	// Recolor pixels
	for (int x = 0; x < _dim.width; ++x) {
		for (int y = 0; y < _dim.height; ++y) {
			cv::Vec3f v = _projId.at<cv::Vec3f>(y,x);
			float s = v[0] / 600.0;
			float t = v[1] / 600.0;
			for (int z = 0; z < 3; ++z) {
				if (v[2] == 1) {
					c.at<cv::Vec3b>(y,x)[z] = 0;
				} else {
					int color = s * (t*r[z] + (1-t)*m[z]) + (1-s)*(t*b[z] + (1-t)*g[z]);
					c.at<cv::Vec3b>(y,x)[z] = color;
				}
			}
		}
	}

	// Draw projector centers
	for (int x = 0; x < 600; ++x) {
		for (int y = 0; y < 600; ++y) {
			cv::Vec3f v = _centers.at<cv::Vec3f>(y,x);
			if (v[2] > 0) {
				int X = v[0], Y = v[1];
				c.at<cv::Vec3b>(Y,X)[0] = 255;
				c.at<cv::Vec3b>(Y,X)[1] = 255;
				c.at<cv::Vec3b>(Y,X)[2] = 255;
			}
		}
	}

	cv::imwrite("colored.jpg", c);
	return c;
}
