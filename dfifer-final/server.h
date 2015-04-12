#ifndef _SERVER_H_
#define _SERVER_H_

#include <vector>
#include <pthread.h>

typedef void (*ServerFunc)(int);

class Server {
 public:
	Server(int socket, ServerFunc func);
	~Server();
	
	// Server accessors
	int fd() { return _fd; }
	int port() { return _port; }
	
	// Create a new thread and runs func
	void run();

	// Execute the provided function on the connected client
	void exec(int fd) { if (_func) (*_func)(fd); } 

 private:
	int _port;
	int _fd;
	ServerFunc _func;
	std::vector<pthread_t> _threads;
};

#endif  // _SERVER_H_
