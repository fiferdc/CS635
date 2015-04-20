#include "server.h"
#include "java_reader.h"
#include "camera.h"
#include "rectify.h"
#include "proj_detector.h"
#include "cornermatcher.h"
#include "feature_matcher.h"

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h> 

const char send_img[] = "send.png";
const char recv_img[] = "recv.png";

const char pattern[] = "BCGYRMBYGRCMRBGCYMGBRYBM";

// Server applications
#define APP_DONE -1
#define CAM_INIT 0
#define RECV_IMG 1
#define CAM_CALIB 2
#define PROJ_CALIB 3

// Language definition
#define LANG_JAVA 0
#define LANG_C 1 

Camera camera(2592, 1944);
Camera projector(600,600);

void test() {
	
	cv::Mat pat = cv::imread("Lenna.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img = cv::imread("colorlenna.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	FeatureMatcher fm(pat, img);
	img = fm.match();

	FeatureMatcher fm2(pat, img);
	fm2.match();
	
	return;

	Rectification r(8,8);
	
	cv::Mat ih = cv::imread("test_h.jpg"), iv = cv::imread("test_v.jpg");
	cv::Mat mh, mv;
	cv::Mat ch, cv;
	Chessboard cbh(ih), cbv(iv);
	pat = cv::imread("normal.jpg");
	Chessboard goodboard(pat);
	std::cout << (goodboard.cornersDetected()?"Corners found":"Corners not found") << std::endl;
	CornerMatcher cm(goodboard, cbv);
	cm.match();

	// Recitfy vertical
	if (r.rectify(iv, cbv, mv)) {
		cv::imwrite("rectified_v.jpg", mv);
	} else {
		printf("Vertical rectification failed\n");
		return;
	}

	
	// Rectify horizontal
	if (r.rectify(ih, cbh, mh)) {
		cv::imwrite("rectified_h.jpg", mh);
	} else {
		printf("Horizontal rectification failed\n");
		return;
	}
	std::vector<Line> lines_h, lines_v;
	FindLines(mv, cv, lines_v);
	FindLines(mh, ch, lines_h);

	std::vector<cv::Point2f> points;
	FindIntersections(lines_h, lines_v, points);

	cv::Mat m;
	cv::addWeighted(mv, 0.5, mh, 0.5, 0.0, m);
	for (auto it = points.begin(); it != points.end(); ++it) {
		circle(m, *it, 3, cv::Scalar(0, 0, 0), -1, 8);
		circle(m, *it, 2, cv::Scalar(255, 255, 255), -1, 8);
	}

	cv::imwrite("ch.jpg", ch);
	cv::imwrite("cv.jpg", cv);
	cv::imwrite("corners.jpg", m);

	std::vector<cv::Point2f> proj_pts = classifyPoints(ch, cv, points, pattern);
	img = ih.clone();
	for (int i = 0; i < points.size(); ++i) {
		if (proj_pts[i] == cv::Point2f(-1,-1)) continue;
		std::cout << points[i] << " " << proj_pts[i] << std::endl;
		cv::Point3f world(points[i].x*25.4/100.0, points[i].y*25.4/100.0, 0.0);
		projector.addPoint(world, proj_pts[i]);
		cv::Point2f p = r.invert(points[i]);
		circle(img, p, 3, cv::Scalar(0, 0, 0), -1, 8);
		circle(img, p, 2, cv::Scalar(255, 255, 255), -1, 8);
	}

	cv::imwrite("final.jpg", img);

	projector.cvCalibrate();
	

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
			// Send type (grey or color)
			msg = 0;
			write(fd, &msg, sizeof(msg));
			
			// Send orientation (vertical or horizontal)
			msg = axis;
			write(fd, &msg, sizeof(msg));

			// Send level
			msg = mask;
			write(fd, &msg, sizeof(msg));

			usleep(4E6);
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
	int sa;
	while ((sa = jr.readInt()) != APP_DONE) { 
		switch(sa) {
			case CAM_INIT:
			{
				camInit(fd);
				break;
			}
			case RECV_IMG:
			{
				getImage(fd);
				test();
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
			default: {}
		}
	}
	jr.close();
}

int main(int argc, char** argv)
{

	test();
	return 1;

	Server camServer(1234, &serverFunc); 
	camServer.run();	
	Server projServer(1235, &structuredLight);
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


	while (1) {}
	return 0;
}
