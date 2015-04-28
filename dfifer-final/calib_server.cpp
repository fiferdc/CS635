#include "server.h"
#include "java_reader.h"
#include "camera.h"
#include "rectify.h"
#include "proj_detector.h"
#include "cornermatcher.h"
#include "feature_matcher.h"
#include "epipolar.h"

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h> 
#include <semaphore.h>
#include <stdio.h>

extern int gui_app(int argc, char *argv[]);


const char send_img[] = "send.png";
const char recv_img[] = "recv.png";

const char pattern[] = "BCGYRMBYGRCMRBGCYMGBRYBM";

// Producer-consumer buffer
// Producer is android app
// Consumer is desktop app
sem_t sem_p;
sem_t sem_c;

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

Camera camera(2592, 1944);
Camera projector(600,600);

void test() {
	
	cv::Mat pat = cv::imread("Lenna.png");
	cv::Mat ih = cv::imread("lenna_h2.jpg");
	cv::Mat iv = cv::imread("lenna_v2.jpg");

	Rectification rh(pat, ih);
	cv::Mat mh = rh.rectify();
	
	Rectification rv(pat, iv);
	cv::Mat mv = rv.rectify();

/*
	return;

	FeatureMatcher fm(pat, img);
	img = fm.match();

	FeatureMatcher fm2(pat, img);
	fm2.match();
	
	return;
*/
	
	
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
	
	std::vector<cv::Point2f> imgPts = rh.getImgPts();
	std::vector<cv::Point2f> patPts = rh.getPatPts();

	std::cout << "imgPts size: " << imgPts.size() << std::endl;
	std::cout << "patPts size: " << patPts.size() << std::endl;

	for (int i = 0; i < imgPts.size(); ++i) {
		cv::Point2f imgPt = imgPts[i];
		cv::Point2f patPt = patPts[i];
		if (imgPt.x != imgPt.x || imgPt.y != imgPt.y) continue;
		cv::Point3f world(patPt.x*7*25.4/512.0, patPt.y*7*25.4/512.0, 0.0);
//		std::cout << world << "->" << imgPt << std::endl;
		camera.addPoint(world, imgPt);
	}

	cv::Mat corners = m.clone();	
	for (auto it = points.begin(); it != points.end(); ++it) {
		circle(corners, *it, 3, cv::Scalar(0, 0, 0), -1, 8);
		circle(corners, *it, 2, cv::Scalar(255, 255, 255), -1, 8);
//		std::cout << *it << std::endl;
	}

	std::cout << "Camera calibration" << std::endl;
	camera.cvCalibrate();

	cv::imwrite("corners.jpg", corners);

	std::vector<cv::Point2f> proj_pts = classifyPoints(ch, cv, points, pattern);
	cv::Mat img = ih.clone();

	// For epipolar geometry
	std::vector<cv::Point3f> worldPts;
	std::vector<cv::Point2f> discoveredPts;
	
	for (int i = 0; i < points.size(); ++i) {
		if (proj_pts[i] == cv::Point2f(-1,-1)) continue;
		if (proj_pts[i].x != proj_pts[i].x) continue;
		std::cout << points[i] << " " << proj_pts[i] << std::endl;
		cv::Point3f world(points[i].x*7*25.4/512.0, points[i].y*7*25.4/512.0, 0.0);
		projector.addPoint(world, proj_pts[i]);
		worldPts.push_back(world);
		discoveredPts.push_back(proj_pts[i]);
		//cv::Point2f p = rh.invert(points[i]);
		//circle(img, p, 3, cv::Scalar(0, 0, 0), -1, 8);
		//circle(img, p, 2, cv::Scalar(255, 255, 255), -1, 8);
	}

	//cv::imwrite("final.jpg", img);
	
	std::cout << "Projector calibration" << std::endl;
	projector.cvCalibrate();

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

	std::cout << "Validation" << std::endl;
	for (int i = 0; i < cam1.size(); ++i) {
		cv::Point2f imgPt = cam1[i];
		cv::Point2f expected = discoveredPts[i];
		cv::Point2f actual = er.findPointInCamera(imgPt, expected.x, 0, 0);
		std::cout << expected << " " << actual << std::endl;
	}

	// Reconstruct geometry


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

void structuredLight(int fd)
{

	int msg;
	int width, height;

	// Read width and height massks
	read(fd, &width, sizeof(width));
	read(fd, &height, sizeof(height));
	
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
	msg = -1;
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
				unsigned char c = 1;
				for (int i = 0; i < 10; ++i) {
					write(fd, &c, sizeof(c));
					sem_wait(&sem_p);
					getImage2(fd);
					sem_post(&sem_c);
					usleep(2500000);
				}
				//write(fd, &c, sizeof(c));
				//getImage2(fd);
				break;
			}
			default: {}
		}
	}
	jr.close();
}

int main(int argc, char** argv)
{

//	test();
//	return 0;

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
