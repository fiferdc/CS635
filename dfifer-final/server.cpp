#include "server.h"

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <vector>

void * _accept(void * s_ptr) {
	Server *server = (Server *)s_ptr;
	sockaddr_in client;
	socklen_t client_len = sizeof(client);
	while (1) {
		int fd = accept(server->fd(), (sockaddr*)&client, &client_len);
		if (fd < 0) {
			perror("ERROR: Accept\n");
			return (void*)1;
		} else {
			printf("\033[1;32m[%s]\033[0m Client connection has been established\n", server->name());
		}
		server->exec(fd);	
		printf("\033[1;32m[%s]\033[0m Session complete\n", server->name());
	}
}

Server::Server(const char* name, int port, ServerFunc func)
		: _port(port), _func(func) {

	_name = strdup(name);
	sockaddr_in server;
	_fd = socket(AF_INET, SOCK_STREAM, 0);
	bzero((char *) &server, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_port = htons(_port);
	server.sin_addr.s_addr = INADDR_ANY;

	if (bind(_fd, (sockaddr*)&server, sizeof(server)) < 0) {
		perror("ERROR: Bind\n");
	}

	listen(_fd, 5);
	printf("\033[1;32m[%s]\033[0m Listening for connections\n", _name);	
}

Server::~Server()
{
	for (auto it = _threads.begin(); it != _threads.end(); ++it) {
		pthread_join(*it, NULL);
	}
	delete(_name);
}

void
Server::run()
{
	pthread_t	thread;
	pthread_create(&thread, NULL, _accept, (void *)this);
	_threads.push_back(thread);
}
