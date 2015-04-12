#include "java_reader.h"

#include <stdio.h>
#include <netinet/in.h>

char *
JavaReader::readString() 
{
	short len;
	char *buf;
	read(_fd, &len, sizeof(len));
	len = ntohs(len);
	buf = new char[len+1];
	read(_fd, buf, len);
	buf[len] = '\0';
	return buf;
}

int
JavaReader::readInt()
{
	int msg;
	read(_fd, &msg, sizeof(msg));
	return ntohl(msg);
}

float
JavaReader::readFloat()
{
	int i = readInt();
	float* f = (float*)&i;	
	return *f;
}

unsigned char
JavaReader::readByte()
{
	unsigned char msg;
	read(_fd, &msg, sizeof(msg));
	return msg;
}

unsigned char* 
JavaReader::readBytes(int numBytes)
{
	unsigned char* buf = new unsigned char[numBytes];
	unsigned char* ptr = buf;
	while (numBytes) {
		int readBytes = read(_fd, ptr, numBytes);
		ptr += readBytes;
		numBytes -= readBytes;
	}
	return buf;
}	
