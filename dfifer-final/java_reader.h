#ifndef _JAVA_READER_H_
#define _JAVA_READER_H_

#include <unistd.h>

class JavaReader {
 public:
  JavaReader(int fd) : _fd(fd) {}
	~JavaReader() {}
	void close() {
		::close(_fd);
	}
	char* readString();
	int readInt();
	float readFloat();
	unsigned char readByte();
	unsigned char* readBytes(int);
 private:
	int _fd;
};

#endif  // _JAVA_READER_H_
