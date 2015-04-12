#include "server.h"

#include <strings.h>
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
			printf("Client connection has been established\n");
		}
		server->exec(fd);	
	}
}

Server::Server(int port, ServerFunc func)
		: _port(port), _func(func) {

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
}

Server::~Server()
{
	for (auto it = _threads.begin(); it != _threads.end(); ++it) {
		pthread_join(*it, NULL);
	}
}

void
Server::run()
{
	pthread_t	thread;
	pthread_create(&thread, NULL, _accept, (void *)this);
	_threads.push_back(thread);
}
