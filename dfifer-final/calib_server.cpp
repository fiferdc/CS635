#include "server.h"
#include "java_reader.h"
#include "camera.h"
#include "rectify.h"
#include "proj_detector.h"
#include "cornermatcher.h"
#include "feature_matcher.h"
#include "epipolar.h"
#include "structured_light.h"

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h> 
#include <semaphore.h>
#include <stdio.h>

extern int gui_app(std::vector<cv::Point3f>);


const char send_img[] = "send.png";
const char recv_img[] = "recv.png";

const char pattern[] = "BCGYRMBYGRCMRBGCYMGBRYBM";

// Producer-consumer buffer
// Producer is android app
// Consumer is desktop app
sem_t sem_p;
sem_t sem_c;

int android_msg = 1;

// Server applications
#define APP_DONE -1
#define CAM_INIT 0
#define RECV_IMG 1
#define CAM_CALIB 2
#define PROJ_CALIB 3
#define CLIENT_TEST 4

// Language definition
#define LANG_JAVA 0
#define LANG_C 1 

int w_iter = 10;
int h_iter = 10;

Camera camera(2592, 1944);
Camera projector(600,600);

void test() {
	

	// Test of pixel intensity
	
	cv::Mat p22 = cv::imread("p20.jpg");
	cv::Mat p23 = cv::imread("p21.jpg");
	cv::cvtColor(p22, p22, CV_BGR2Lab);
	cv::cvtColor(p23, p23, CV_BGR2Lab);
	cv::Mat slImg(p22.size(), CV_32FC1);


	StructuredLight sl(p22.size());
	for (int i = 2; i < 22; i += 2) {
		char filename[10];
		sprintf(filename, "p%i.jpg", i);
		cv::Mat on = cv::imread(filename);
		sprintf(filename, "p%i.jpg", i+1);
		cv::Mat off = cv::imread(filename);
		sl.addImagePair(on, off, 0);
	}
	for (int i = 24; i < 44; i += 2) {
		char filename[10];
		sprintf(filename, "p%i.jpg", i);
		cv::Mat on = cv::imread(filename);
		sprintf(filename, "p%i.jpg", i+1);
		cv::Mat off = cv::imread(filename);
		sl.addImagePair(on, off, 1);
	}
	sl.decode();
	sl.getColored();

	/*
	for (int x = 0; x < p22.cols; ++x) {
		for (int y = 0; y < p22.rows; ++y) {
			float f1 = p22.at<cv::Vec3b>(y,x)[0];
			float f2 = p23.at<cv::Vec3b>(y,x)[0];
			float f = 0.5;

			if (abs(f1-f2)/255.0 > 0.01) {
				f += (f1 > f2)?0.5:-0.5;
			}
			slImg.at<float>(y,x) = f*255;
		}
	}
	cv::imwrite("sltest.jpg", slImg);
	return;
	*/

	// Calibrate camera and projector

	cv::Mat pat = cv::imread("Lenna.png");
//	cv::Mat ih = cv::imread("lenna_h.jpg");
//	cv::Mat iv = cv::imread("lenna_v.jpg");
	cv::Mat good = cv::imread("c0.jpg");

	FeatureMatcher fm(pat, good);
	cv::imwrite("warp.jpg", fm.match());

	Rectification r(pat, good);
	r.rectify();

	cv::Mat ih = cv::imread("c1.jpg");
	cv::Mat iv = cv::imread("c2.jpg");
/*
	for (int x = 0; x < mask.cols; ++x) {
		for (int y = 0; y < mask.rows; ++y) {
			if (mask.at<float>(y,x) == 0) {
				for (int z = 0; z < 3; ++z) {
					ih.at<cv::Vec3b>(y,x)[z] = 0;
					iv.at<cv::Vec3b>(y,x)[z] = 0;
				}
			}
		}
	}

	cv::imwrite("ih.jpg", ih);
	cv::imwrite("iv.jpg", iv);

	Rectification rh(pat, ih);
	cv::Mat mh = rh.rectify();
	
	Rectification rv(pat, iv);
	cv::Mat mv = rv.rectify();
*/
	cv::Mat mh = r.applyRectification(ih);
	cv::Mat mv = r.applyRectification(iv);
	
	cv::Mat ch, cv;
	std::vector<Line> lines_h, lines_v;

	FindLines(mh, ch, lines_h);
	cv::imwrite("ch.jpg", ch);
	
	FindLines(mv, cv, lines_v);
	cv::imwrite("cv.jpg", cv);

	std::vector<cv::Point2f> points;
	FindIntersections(lines_h, lines_v, points);

	cv::Mat m;
	cv::addWeighted(mv, 0.5, mh, 0.5, 0.0, m);
	
	std::vector<cv::Point2f> imgPts = r.getImgPts();
	std::vector<cv::Point2f> patPts = r.getPatPts();

	std::cout << "imgPts size: " << imgPts.size() << std::endl;
	std::cout << "patPts size: " << patPts.size() << std::endl;

	for (int i = 0; i < imgPts.size(); ++i) {
		cv::Point2f imgPt = imgPts[i];
		cv::Point2f patPt = patPts[i];
		if (imgPt.x != imgPt.x || imgPt.y != imgPt.y) continue;
//		imgPt.y = good.rows - imgPt.y;
//		cv::Point3f world(patPt.x*7*25.4/512.0, (512.0-patPt.y)*7*25.4/512.0, 0.0);
		cv::Point3f world(patPt.x*7*25.4/512.0, patPt.y*7*25.4/512.0, 0.0);
//		std::cout << world << "->" << imgPt << std::endl;
		camera.addPoint(world, imgPt);
	}

	cv::Mat corners = m.clone();	

	std::cout << "Camera calibration" << std::endl;
	camera.cvCalibrate();
	camera.printError("cam_err.txt");

	std::vector<cv::Point2f> proj_pts = classifyPoints(ch, cv, points, pattern);
	cv::Mat img = ih.clone();

	// For epipolar geometry
	std::vector<cv::Point3f> worldPts;
	std::vector<cv::Point2f> discoveredPts;
	std::vector<int> goodPoints;

	for (int i = 0; i < points.size(); ++i) {
		if (proj_pts[i] == cv::Point2f(-1,-1)) continue;
		if (proj_pts[i].x != proj_pts[i].x) continue;
		goodPoints.push_back(i);
	}


	// Find valid mappings
	std::vector<unsigned char> mask;
	{
		std::vector<cv::Point2f> srcPts;
		std::vector<cv::Point2f> dstPts;
		for (int j = 0; j < goodPoints.size(); j++) {
			int i = goodPoints[j];
			srcPts.push_back(points[i]);
			dstPts.push_back(proj_pts[i]);
		}
		cv::findHomography(srcPts, dstPts, CV_RANSAC, 3, mask);
	}

	for (int j = 0; j < goodPoints.size(); j++) {
		int i = goodPoints[j];
		if (mask[i] == 0) continue;
		circle(corners, points[i], 3, cv::Scalar(0, 0, 0), -1, 8);
		circle(corners, points[i], 2, cv::Scalar(255, 255, 255), -1, 8);

		std::cout << points[i] << " " << proj_pts[i] << std::endl;
//		proj_pts[i].y = 600 - proj_pts[i].y;
//		cv::Point3f world(points[i].x*7*25.4/512.0, (512.0-points[i].y)*7*25.4/512.0, 0.0);
		cv::Point3f world(points[i].x*7*25.4/512.0, points[i].y*7*25.4/512.0, 0.0);
		projector.addPoint(world, proj_pts[i]);
		worldPts.push_back(world);
		discoveredPts.push_back(proj_pts[i]);
		//cv::Point2f p = rh.invert(points[i]);
		//circle(img, p, 3, cv::Scalar(0, 0, 0), -1, 8);
		//circle(img, p, 2, cv::Scalar(255, 255, 255), -1, 8);
	}
	//cv::imwrite("final.jpg", img);

	cv::imwrite("corners.jpg", corners);
	
	std::cout << "Projector calibration" << std::endl;
	projector.cvCalibrate();
	projector.printError("proj_err.txt");

	// Construct epipolar relationship between the camera and projector

	std::vector<cv::Point2f> cam1, cam2;
	for (auto it = worldPts.begin(); it != worldPts.end(); ++it) {
		cam1.push_back(camera.project(*it));
		cam2.push_back(projector.project(*it));
	}
	
	EpipolarRelation er(cam1, cam2);

	cv::Mat validation = ih.clone();
	for (auto it = cam1.begin(); it != cam1.end(); ++it) {
		circle(validation, *it, 3, cv::Scalar(0, 0, 0), -1, 8);
		circle(validation, *it, 2, cv::Scalar(255, 255, 255), -1, 8);
	}
	cv::imwrite("validation.jpg", validation);

	//std::cout << "Validation" << std::endl;
	/*for (int i = 0; i < cam1.size(); ++i) {
		cv::Point2f imgPt = cam1[i];
		cv::Point2f expected = discoveredPts[i];
		cv::Point2f actual = er.findPointInCamera(imgPt, expected.x, 0, 0);
		std::cout << expected << " " << actual << std::endl;
	}*/

	// Reconstruct geometry

	cv::Mat	centers = sl.getCenters();
	std::vector<cv::Point3f> reprojection;
	std::vector<cv::Vec3b> colors;
	std::cout << "Reprojection" << std::endl;
	for (int x = 0; x < 7; ++x) {
		for (int y = 0; y < 7; ++y) {
			float X = x*25.4;
			float Y = y*25.4;
			cv::Point3f w(X, Y, 0);
			reprojection.push_back(w);
		}
	}

	// Also draw scene from projector's view
	cv::Mat invImg = cv::Mat::zeros(600, 600, CV_8UC3);
	cv::Mat colored = cv::imread("p0.jpg");
	for (int x = 0; x < 600; ++x) {
		for (int y = 0; y < 600; ++y) {
			cv::Vec3f v = centers.at<cv::Vec3f>(y,x);
			if (v[2] > 0) {
				float X = v[0], Y = v[1];
				cv::Point2f p0(x,y);
				cv::Point2f p1(X,Y);
				invImg.at<cv::Vec3b>(y,x) = colored.at<cv::Vec3b>(Y,X);
				cv::Point3f p = projector.reproject(p0, camera, p1);
				reprojection.push_back(p);
				cv::Vec3b color = colored.at<cv::Vec3b>(Y,X);
				colors.push_back(colored.at<cv::Vec3b>(Y,X));
//				std::cout << p0 << "<=>" << p1 << std::endl;
//				std::cout << p << std::endl;
			}
		}
	}
	cv::imwrite("invImg.jpg", invImg);

	gui_app(reprojection);


}


void sendImage(int fd)
{
	int len;
	cv::Mat m = cv::imread(send_img);
	int rows = m.rows;
	int cols = m.cols;
	int type = m.type();

	// Send opencv data
	printf("Sending %d x %d matrix of type %d\n", rows, cols, type);
	write(fd,	&rows, sizeof(rows));
	write(fd, &cols, sizeof(cols));
	write(fd, &type, sizeof(type));

	// Send filename
	len = strlen(recv_img)+1;
	write(fd, &len, sizeof(int));
	write(fd, recv_img, len);


	// Send data
	len = rows * cols * m.elemSize();
	printf("Sending %d bytes of data\n", len);
	write(fd, &len, sizeof(int));
	len = write(fd, m.data, len);
	printf("Sent %d bytes of data\n",len);
	
	// Done
	close(fd);
}

void getImage(int fd)
{
	int len;
	int rows, cols, type;
	JavaReader jr(fd);

	// Read opencv data
	//read(fd, &rows, sizeof(rows));
	//read(fd, &cols, sizeof(cols));
	//read(fd, &type, sizeof(type));
	rows = jr.readInt();
	cols = jr.readInt();
	type = jr.readInt();
	printf("Read %d x %d matrix of type %d\n", rows, cols, type);

	// Read filename
	//read(fd, &len, sizeof(len));
	//char *filename = new char[len];
	//read(fd, filename, len);
	char *filename = jr.readString();
	printf("Will store image in file %s\n", filename);

	// Read data
	//read(fd, &len, sizeof(len));
	len = jr.readInt();
	printf("Reading %d bytes of data\n", len);
	/*unsigned char *data = new unsigned char[len];
	unsigned char *ptr = data;
	int bytes_read;
	while ((bytes_read = read(fd, ptr, len)) > 0) {
		ptr += bytes_read;
	}*/
	unsigned char *data = jr.readBytes(len);
	//printf("Read %d bytes of data\n", ptr-data);

	// Save image
	cv::Mat m(rows, cols, type, data);	
	cv::imwrite(filename, m);

	// Done
}

void getImage2(int fd)
{
	JavaReader jr(fd);
	char *filename = jr.readString();
	printf("Will store image in file %s\n", filename);
	int len = jr.readInt();
	printf("Reading %d bytes of data\n", len);
	unsigned char *data = jr.readBytes(len);
	FILE *img = fopen(filename, "wb");
	fwrite(data, 1, len, img);
	printf("Done!\n");
	fclose(img);
}

// Calibrates camera and projector
void beginCalibration(int fd)
{

	int msg;


	// White Image
		sem_wait(&sem_c);
		printf("Sending data to projector...\n");
		// Send type color
		msg = 0;
		write(fd, &msg, sizeof(msg));
		
		// Send orientation (doesn't matter)
		msg = 0;
		write(fd, &msg, sizeof(msg));

		// Send level (big number!)
		msg = 20000;
		write(fd, &msg, sizeof(msg));

		printf("Waiting on response from projector...\n");
		read(fd, &msg, sizeof(msg));
		sem_post(&sem_p);

	// Colored Stripes	
	for (int axis = 0; axis < 2; ++axis) {
			sem_wait(&sem_c);
			printf("Sending data to projector...\n");
			// Send type color
			msg = 2;
			write(fd, &msg, sizeof(msg));
			
			// Send orientation (vertical or horizontal)
			msg = axis;
			write(fd, &msg, sizeof(msg));

			// Send level (doesn't matter)
			msg = 0;
			write(fd, &msg, sizeof(msg));

			printf("Waiting on response from projector...\n");
			read(fd, &msg, sizeof(msg));
			sem_post(&sem_p);
	}
	usleep(10000000);
}

void structuredLight(int fd)
{
	int msg;
	int width, height;

	// Read width and height masks
	read(fd, &w_iter, sizeof(width));
	read(fd, &h_iter, sizeof(height));
	
	w_iter;
	h_iter;

	width = 1 << (w_iter);
	height = 1 << (h_iter);

	w_iter++;
	h_iter++;
	
	// Initialize calibration
	beginCalibration(fd);


	printf("Projection size: %dx%d\n", width, height);
	fflush(stdout);

	for (int axis = 0; axis < 2; ++axis) {
		int mask = axis?height:width;
		
		while (mask) {
			for (int type = 0; type < 2; ++type) {
				sem_wait(&sem_c);
				printf("Sending data to projector...\n");
				// Send type (grey or color)
				msg = type;
				write(fd, &msg, sizeof(msg));
				
				// Send orientation (vertical or horizontal)
				msg = axis;
				write(fd, &msg, sizeof(msg));

				// Send level
				msg = mask;
				write(fd, &msg, sizeof(msg));

				printf("Waiting on response from projector...\n");
				read(fd, &msg, sizeof(msg));
				sem_post(&sem_p);
			}
			mask >>= 1;
		}
	}
	android_msg = 0;
	msg = -1;
	usleep(1000000);
	write(fd, &msg, sizeof(msg));
}


void printText(int fd)
{
	JavaReader jr(fd);
	char * buf = jr.readString();
	printf("%s\n", buf);
	delete [] buf;
}

void camInit(int fd)
{
	JavaReader jr(fd);
	float f = jr.readFloat();
	float dx = jr.readFloat();
	float dy = jr.readFloat();
	printf("Focal length: %lf\n", f);
	printf("Pixel width: %lf\n", dx);
	printf("Pixel height: %lf\n", dy);
	camera.setInternals(f, dx, dy);
}

void camCalib(int fd)
{
	cv::Mat m = cv::imread("test.jpg");
//	camera.calibrate(m);
	camera.cvCalibrate();
}

void serverFunc(int fd) {
	JavaReader jr(fd);
	unsigned char lang = jr.readByte();
	printf("Language: %d\n", lang);
	int sa;
	while ((sa = jr.readInt()) != APP_DONE) { 
		printf("Server Application: %d\n", sa);
		switch(sa) {
		case CAM_INIT:
			{
				camInit(fd);
				break;
			}
			case RECV_IMG:
			{
				getImage2(fd);
				break;
			}
			case CAM_CALIB:
			{
				camCalib(fd);
				break;
			}
			case PROJ_CALIB:
			{
				break;
			}
			case CLIENT_TEST:
			{
				usleep(1000000);
				unsigned char c = 1;
				//for (int i = 0; i < 10; ++i) {
				for (int i = 0; i < 3 + 2*w_iter + 2*h_iter; ++i) {
//				do {	
					sem_wait(&sem_p);
					write(fd, &c, sizeof(c));
					getImage2(fd);
					sem_post(&sem_c);
					usleep(500000);
	//			} while (android_msg);
				}
				c = android_msg;
				write(fd, &c, sizeof(c));
				//write(fd, &c, sizeof(c));
				//getImage2(fd);
				test();
				break;
			}
			default: {}
		}
	}
	jr.close();
}

int main(int argc, char** argv)
{
	
	//test();
	//return 0;

	sem_init(&sem_p, 0, 0);
	sem_init(&sem_c, 0, 1);

	Server camServer("Android", 1234, &serverFunc); 
	camServer.run();	
	Server projServer("Projector", 1235, &structuredLight);
	projServer.run();
/*	hostent * server = gethostbyname("localhost");
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in serv_addr;
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, 
			(char *)&serv_addr.sin_addr.s_addr,
			server->h_length);
	serv_addr.sin_port = htons(projServer.port());
	if (connect(fd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
		perror("ERROR connecting");		
	sendImage(fd);*/

	while(1) {}
	return 0;
}
