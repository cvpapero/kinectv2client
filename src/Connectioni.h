#pragma once
#include <iostream>
#include "zmq.hpp"
class Connection{
private:
	int m_thread;
public:
	Connection();
	Connection( const int threads );
	zmq::socket_t *socket;

	void start();
	void send( const std::string message );
	void recv( std::string &messsage );
	int getNumThreads();
private:
	void initialize();
};