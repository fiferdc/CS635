#include "line.h"

#include "DBSCAN/dbscan.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

void
relabel(float** actual, cv::Mat centers, int* relabeled, int count=6)
{
	std::list<int> labels;
	for (int i = 0; i < count; ++i) {
		labels.push_back(i);
	}
	
	float c[count][3];// = new float[count][3];
	for (int i = 0; i < count; ++i) {
		for (int j = 0; j < 3; ++j) {
			c[i][j] = centers.at<float>(i, j);
		}		
	}
	
	for (int i = 0; i < count; ++i) {
		int match = -1;
		float min_err = 100000;
		std::cout << "Matching [ "
			 << actual[i][0] << ", "
			 << actual[i][1] << ", "
			 << actual[i][2] << " ]\n";
		for (auto it = labels.begin(); it != labels.end(); ++it) {
			int l = *it;
			float dx = c[l][0] - actual[i][0];
			float dy = c[l][1] - actual[i][1];
			float dz = c[l][2] - actual[i][2];
			float err = dx*dx + dy*dy + dz*dz;
			if (err < min_err) {
				min_err = err;
				match = l;
			}
		}
		std::cout << i << "->" << match << std::endl;
		relabeled[match] = i;
		labels.remove(match);
	}
}

bool
FindLines(const cv::Mat& in, cv::Mat& clusters, std::vector<Line>& out)
{
	cv::Mat lab, yuv;

	cv::Size dsize = in.size();
	cv::Size _grid(8,8);
	
	cv::cvtColor(in, lab, CV_BGR2YUV);
	cv::imwrite("lab.jpg", lab);

	
	cv::Mat blur;
	cv::blur(lab, blur, cv::Size(5,5));
	cv::imwrite("blur.jpg", blur); 
	lab = blur.clone();

	/*std::vector<cv::Mat> Lab;
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
	

	// Adjust Lab image
	cv::merge(&Lab[0], 3, lab);
	cv::imwrite("ab.jpg", lab);
	cv::imwrite("a.jpg", Lab[1]);
	cv::imwrite("b.jpg", Lab[2]);

	cv::Mat ab;
	cv::merge(&Lab[1], 2, ab);
*/
	cv::Mat samples(dsize.width*dsize.height, 3, CV_32F);
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			samples.at<float>(y + x*dsize.height, 0) = 0;
			for (int z = 0; z < 2; ++z) {
				samples.at<float>(y + x*dsize.height, z+1) = lab.at<cv::Vec3b>(y,x)[z+1];
			}
		}
	}

	// Cluster segments (6 colors + black squares)
	cv::Mat labels, centers;
	cv::kmeans(samples, 6, labels, 
			cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.001),
			3, cv::KMEANS_PP_CENTERS, centers);

	// Reassign labels to expected colors
	float black[3] = {0, 0, 0};

	float lab_r[3] = {0, 80, 67};
	float lab_y[3] = {0, 255-21, 94};
	float lab_g[3] = {0, 255-86, 83};
	float lab_c[3] = {0, 255-48, 255-14};
	float lab_b[3] = {0, 79, 255-107};
	float lab_m[3] = {0, 98, 255-60};
	float *lab_colors[6] = {
		lab_r, lab_y, lab_g, lab_c, lab_b, lab_m
	};

	float rgb_r[3] = {255, 0, 0};
	float rgb_y[3] = {255, 255, 0};
	float rgb_g[3] = {0, 255, 0};
	float rgb_c[3] = {0, 255, 255};
	float rgb_b[3] = {0, 0, 255};
	float rgb_m[3] = {255, 0, 255};
	float *rgb_colors[6] = {
		rgb_r, rgb_y, rgb_g, rgb_c, rgb_b, rgb_m
	};

	int relabeled[6];
	cv::Mat rgbCenters = cv::Mat::zeros(centers.size(), CV_32F);
	int lcount[6] = {0,0,0,0,0,0};
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			int label = labels.at<int>(y+dsize.height*x,0);
			for (int z= 0; z < 3; ++z) {
				float c = in.at<cv::Vec3b>(y,x)[z];
				rgbCenters.at<float>(label,z) += c;			
			}
			lcount[label]++;
		}
	}
	for (int i = 0; i < 6; ++i) {
		for (int z= 0; z < 3; ++z) {
			rgbCenters.at<float>(i,z) /= lcount[i];		
		}
	}
	
	std::cout << "Cluster colors\n";
	std::cout << rgbCenters << std::endl;


	relabel(rgb_colors, rgbCenters, relabeled);
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 3; ++j) {
			centers.at<float>(i, j) = rgb_colors[relabeled[i]][j];
		}		
	}
	
	std::cout << "Cluster new colors\n";
	std::cout << centers << std::endl;

	// Recolor image
	clusters = cv::Mat(dsize, CV_8UC3);
	for (int x = 0; x < dsize.width; ++x) {
		for (int y = 0; y < dsize.height; ++y) {
			int label = labels.at<int>(y+dsize.height*x,0);
			clusters.at<cv::Vec3b>(y,x)[0] = centers.at<float>(label, 0);
			clusters.at<cv::Vec3b>(y,x)[1] = centers.at<float>(label, 1);
			clusters.at<cv::Vec3b>(y,x)[2] = centers.at<float>(label, 2);
		}
	}

	cv::imwrite("clusters.jpg", clusters);
	
	
	// Perform Canny edge detector
	cv::Mat canny;
	cv::Canny(clusters, canny, 50, 200, 3);

	// Remove natural edges
	/*
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
	*/
	cv::imwrite("canny.jpg", canny);

	// Detect lines
	cv::Mat dlines = clusters.clone();
#if 0
	std::vector<cv::Vec2f> lines;
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
#else 
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(canny, lines, 3, CV_PI/180, 50, 450, 400);

	// From http://opencvexamples.blogspot.com/2013/10/line-detection-by-hough-line-transform.html
	out.clear();
	std::cout << "Lines detected:\n";

	// Find and ignore outlier lines (leave only the expected stripes)

	double avg_theta = 0.0;
	std::vector<cv::Point> pts1, pts2;
	std::vector<double> theta;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		cv::Point pt1(lines[i][0], lines[i][1]), pt2(lines[i][2], lines[i][3]);
		pts1.push_back(pt1);
		pts2.push_back(pt2);
		cv::Point2f p(pt2.x-pt1.x, pt2.y-pt1.y);
		double t = fmod(360 + atan2(p.y, p.x) * 180.0 / CV_PI, 360.0);
		theta.push_back(t);
		avg_theta += t;
	}
	avg_theta /= lines.size();

	double stddev = 0.0;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		double dx = theta[i] - avg_theta;
		stddev += dx*dx;
	}
	stddev = sqrt(stddev / lines.size());


	for( size_t i = 0; i < lines.size(); i++ )
	{
//		cv::Point pt1(lines[i][0], lines[i][1]), pt2(lines[i][2], lines[i][3]);
		cv::Point pt1 = pts1[i], pt2 = pts2[i];
		std::cout << pt1 << " " << pt2 << std::endl;
		if (abs(theta[i] - avg_theta) < 1.5 * stddev) {
		out.push_back(Line(pt1, pt2));
		line( dlines, pt1, pt2, cv::Scalar(0,0,0), 3, CV_AA);
		line( dlines, pt1, pt2, cv::Scalar(255,255,255), 1, CV_AA);
		}
	}
	std::cout << std::endl;
#endif
	cv::imwrite("detected.jpg", dlines);

	return true;
}

void
FindIntersections(const std::vector<Line>& h, const std::vector<Line>& v, std::vector<cv::Point2f>& ret)
{

	ret.clear();
	std::vector<cv::Point2f> pts;

	// Comput the intersections
	for (std::vector<Line>::const_iterator it = h.cbegin(); it != h.cend(); it++) {
		for (std::vector<Line>::const_iterator it2 = v.cbegin(); it2 != v.cend(); it2++) {
			cv::Point2f i;
			if (it->intersect(*it2, i)) {
					pts.push_back(i);
			}
		}
	}

	// Cluster intersections
	clustering::DBSCAN::ClusterData cl_d(pts.size(), 2);
	for (int i = 0; i < pts.size(); ++i) {
		cl_d(i, 0) = pts[i].x;
		cl_d(i, 1) = pts[i].y;
	}
	clustering::DBSCAN dbs(0.02, 1, 4);
	dbs.fit(cl_d);

	clustering::DBSCAN::Labels labels = dbs.get_labels();
	std::map<int, std::vector<cv::Point2f> > clusters;

	for (int i = 0; i < pts.size(); ++i) {
		int l = labels[i];
		clusters[l].push_back(pts[i]);
		//ret.push_back(pts[i]);
	}

	// Find the center of each cluster
	for (auto it = clusters.begin(); it != clusters.end(); ++it) {
		auto cluster = it->second;
		double x=0.0, y=0.0;
		for (auto p = cluster.begin(); p != cluster.end(); ++p) {
			x += p->x;
			y += p->y;
		}
		x /= cluster.size();
		y /= cluster.size();
		ret.push_back(cv::Point2f(x, y));
	}
}

// Returns a list of projector pixels
// (-1, -1) means skip correspondence
std::vector<cv::Point2f>
classifyPoints(
		cv::Mat h, cv::Mat v, std::vector<cv::Point2f> pts,
		const char * pattern)
{

	std::cout << pattern << std::endl;

	std::vector<cv::Point2f> proj_pts;
	proj_pts.clear();

	// In BGR notation
	cv::Vec3b black(0,0,0);
	cv::Vec3b red(0,0,255);
	cv::Vec3b yellow(0,255,255);
	cv::Vec3b green(0,255,0);
	cv::Vec3b cyan(255,255,0);
	cv::Vec3b blue(255,0,0);
	cv::Vec3b magenta(255,0,255);

	for (auto p = pts.begin(); p != pts.end(); ++p) {
		char e[3];
		e[2] = '\0';
		bool valid = true;
		cv::Vec3b dc[2];
		int x,y;

		const int r = 10;

		if (p->x < 0 || p->x >= h.cols || p->y < 0 || p->y >= h.rows) {
			valid = false;
		} else {
			// Process horizontal stripes
			if (h.at<cv::Vec3b>(p->y, p->x) == black) {
				valid = false;
			}
			dc[0] = h.at<cv::Vec3b>(p->y-r, p->x);
			dc[1] = h.at<cv::Vec3b>(p->y+r, p->x);

			// Pattern is reversed this way
			for (int i = 0; i < 2; ++i) {
				if (dc[i] == red)
					e[1-i] = 'R';
				else if (dc[i] == yellow)
					e[1-i] = 'Y';
				else if (dc[i] == green)
					e[1-i] = 'G';
				else if (dc[i] == cyan)
					e[1-i] = 'C';
				else if (dc[i] == blue)
					e[1-i] = 'B';
				else if (dc[i] == magenta)
					e[1-i] = 'M';
				else
					e[1-i] = 'X';
			}

			const char *m = strstr(pattern, e);
			if (m == NULL) valid = false;
			y = 25*(1+m-pattern);

			// Process vertical stripes
			if (v.at<cv::Vec3b>(p->x, p->y) == black) {
				valid = false;
			}

			dc[0] = v.at<cv::Vec3b>(p->y, p->x-r);
			dc[1] = v.at<cv::Vec3b>(p->y, p->x+r);


			for (int i = 0; i < 2; ++i) {
				if (dc[i] == red)
					e[i] = 'R';
				else if (dc[i] == yellow)
					e[i] = 'Y';
				else if (dc[i] == green)
					e[i] = 'G';
				else if (dc[i] == cyan)
					e[i] = 'C';
				else if (dc[i] == blue)
					e[i] = 'B';
				else if (dc[i] == magenta)
					e[i] = 'M';
				else 
					e[i] = 'X';
			}

			m = strstr(pattern, e);
			if (m == NULL) valid = false;
			x = 25*(1+m-pattern);
		}		
		if (valid)
			proj_pts.push_back(cv::Point2f(x,y));
		else 
			proj_pts.push_back(cv::Point2f(-1,-1));
	}

	return proj_pts;
}

