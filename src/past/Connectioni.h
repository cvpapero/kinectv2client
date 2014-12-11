#pragma once
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "zmq.hpp"
class Connection{
private:
	int m_thread;
  	ros::NodeHandle nh;
  	image_transport::ImageTransport it;
  	image_transport::Publisher color_pub;
  	//image_transport::Publisher depth_pub;
  	//image_transport::Publisher bodyindex_pub;
  	ros::Publisher kinectv2_pub;
public:
	Connection();
	Connection( const int threads );
	zmq::socket_t *socket;

	void start();
	void send( const std::string message );
	void recv( std::string &messsage );
	int getNumThreads();
	void test();	
//private:
//	void initialize();

};
