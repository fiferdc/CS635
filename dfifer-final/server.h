#ifndef _SERVER_H_
#define _SERVER_H_

#include <vector>
#include <pthread.h>

typedef void (*ServerFunc)(int);

class Server {
 public:
	Server(const char *name, int socket, ServerFunc func);
	~Server();
	
	// Server accessors
	int fd() const { return _fd; }
	int port() const { return _port; }
	char* name() const { return _name; }
	
	// Create a new thread and runs func
	void run();

	// Execute the provided function on the connected client
	void exec(int fd) { if (_func) (*_func)(fd); } 

 private:
	char* _name;
	int _port;
	int _fd;
	ServerFunc _func;
	std::vector<pthread_t> _threads;
};

#endif  // _SERVER_H_
