/*
やること：tf2に移行する
*/
 
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//#include <sensor_msgs/CameraInfo.h>
//#include <camera_info_manager/camera_info_manager.h>

//#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <humans_msgs/Humans.h>
#include "Util.h"
#include "MessagePackUtility.hpp"
#include "KinectPackUtil.hpp"
#include "KinectPack.h"
#include "zmq.hpp"
#include "JsonToMsg.hpp"
using namespace std;
class Connection
{
private:
  int m_thread;
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  //ros::NodeHandle comm_nh;
  //  camera_info_manager::CameraInfoManager info_mgr;

  image_transport::ImageTransport it;
  image_transport::Publisher color_pub;
  image_transport::Publisher depth_pub;
  image_transport::Publisher bodyindex_pub;
  ros::Publisher kinectv2_pub;
  //  ros::Publisher info_pub;

  //tf::TransformBroadcaster br;
  tf2_ros::TransformBroadcaster br;


  //tf::Transform transform;
  zmq::socket_t *socket;
  string ipaddress;
  string port;
  string c_frame;//, p_frame;
  string url;
  double px, py, pz, roll, pich, yaw;

public:
  Connection()
    :it(nh), pnh("~")//, info_mgr( nh, "camera" )
  {
    cout << __FUNCTION__ << endl;
    pnh.param<std::string>("ipaddress", ipaddress, "133.19.23.90") ;
    pnh.param<std::string>("port", port, "7745");
    pnh.param<std::string>("c_frame", c_frame, "camera_link");
    //pnh.param<std::string>("c_frame", c_frame, "camera_frame");
    //pnh.param<double>("px", px, 0.1);
    //pnh.param<double>("py", py, 0);
    //pnh.param<double>("pz", pz, 1.0);
    //pnh.param<double>("roll", roll, 0);
    //pnh.param<double>("pich", pich, -0.087);
    //pnh.param<double>("yaw", yaw, 0);
    //pnh.getParam("camera_info_url", url);
    //info_mgr.setCameraName( c_frame );
    //info_mgr.loadCameraInfo( url );

    string color_topic, depth_topic, bodyindex_topic;//, info_topic;
    color_topic = "/" + c_frame + "/image/color";
    depth_topic = "/" + c_frame + "/image/depth";
    bodyindex_topic = "/" + c_frame + "/image/bodyindex";
    //info_topic = "/" + c_frame + "/camera_info";
    color_pub = it.advertise( color_topic, 1 );
    depth_pub = it.advertise( depth_topic, 1 );
    bodyindex_pub = it.advertise( bodyindex_topic, 1 );
    //info_pub = nh.advertise<sensor_msgs::CameraInfo>( info_topic, 1 );
    kinectv2_pub = nh.advertise<humans_msgs::Humans>("/humans/KinectV2",10);
    try
      {
	zmq::context_t *conetext = new zmq::context_t( 10 );
	socket = new zmq::socket_t( *conetext, ZMQ_REQ );
	try
	  {
	    cout << "connect:" << ipaddress + ":" + port << endl;
	    socket->connect( ("tcp://" + ipaddress + ":" + port).c_str() );
	  }
	catch ( const zmq::error_t &e )
	  {
	    cout << ("tcp://" + ipaddress + ":" + port).c_str() << endl;
	    cerr << __FUNCTION__ << ":" << e.what() << endl;
	  }
      }
    catch( const zmq::error_t &e )
      {
	cerr << __FUNCTION__ << ":" << e.what() << endl;
      }
  }
  void start()
  {
  }
  void send( const string message )
  {
    zmq::message_t message_send( message.size() );
    memcpy( message_send.data(), message.data(), message.size() );
    socket->send( message_send );
  }
  void recv( string &message )
  {
    zmq::message_t message_recv;
    socket->recv( &message_recv );
    message = 
      std::string(static_cast<char*>(message_recv.data()), message_recv.size());
  }


  void test()
  {
    while( ros::ok() )
      {
	// REP-REQ形式なので要求メッセージを送信する
	Connection::send("test message");
	// 現在はどんな文字列を送っても違いはない
	// データを受信する
	string message;
	Connection::recv( message ); // サーバから帰ってくる文字列
	//cout << "recv" <<endl;
	// メッセージを指定形式(今回はKinectPack)に変換
	// 送られてくるデータ形式と異なるとエラー
	KinectPack kinectPack;
	MsgPackUtil::unpack( message, kinectPack );
	//std::cout << kinectPack.bodies.jsonBodyInfo << std::endl;
	// 文字列をMat型に変換
	cv::Mat imageColor;
	try
	  {
	    KinectPackUtil::convertImageToMat( kinectPack.imageColor, imageColor );
	  }
	catch( const cv::Exception &e )
	  {
	    cout << "Color:" << e.what() << endl;
	  }
	cv::Mat imageDepth;
	try
	  {
	    KinectPackUtil::convertImageToMat( kinectPack.imageDepth, imageDepth );
	  }
	catch( const cv::Exception &e )
	  {
	    cout << "Depth:" << e.what() << endl;
	  }
	cv::Mat imageBodyIndex;
	try
	  {
	    KinectPackUtil::convertImageToMat( kinectPack.imageBodyIndex, imageBodyIndex );
	  }
	catch( const cv::Exception &e )
	  {
	    cout << "BodyIndex:" << e.what() << endl;
	  }
	//jsonからROSmessage
	humans_msgs::Humans kinect_msg;
	double cols_scale = (float)imageColor.cols / (float)imageDepth.cols;
	double rows_scale = (float)imageColor.rows / (float)imageDepth.rows;
	JsonToMsg::body(nh,
			kinectPack, &kinect_msg, 
			cols_scale, rows_scale, c_frame);
	// 検知した人間のJoint情報を元にスケルトンを描画
	//KinectPackUtil::drawSkeleton( imageColor, kinectPack, 
	//cols_scale, rows_scale);
	
	cv::waitKey(1);
	cv_bridge::CvImage cv_img_color, cv_img_depth, cv_img_bodyindex;

	ros::Time time = ros::Time::now();

	cv_img_color.header.stamp = time;
	cv_img_color.header.frame_id = c_frame;
	cv_img_color.encoding = "bgr8";
	cv_img_color.image = imageColor;
	
	cv_img_depth.header.stamp = time;
	cv_img_depth.header.frame_id = c_frame;
	cv_img_depth.encoding = "16UC1";
	cv_img_depth.image = imageDepth;
	
	cv_img_bodyindex.header.stamp = time;
	cv_img_bodyindex.header.frame_id = c_frame;
	cv_img_bodyindex.encoding = "mono8";
	cv_img_bodyindex.image = imageBodyIndex;

	color_pub.publish( cv_img_color.toImageMsg() );
	depth_pub.publish( cv_img_depth.toImageMsg() );
	bodyindex_pub.publish( cv_img_bodyindex.toImageMsg() );

	kinect_msg.header.stamp = time;
	kinect_msg.header.frame_id = c_frame;
	kinectv2_pub.publish( kinect_msg );

	/*
	transform.setOrigin(tf::Vector3(0, 0, 0));
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					      camera_frame+"_child", camera_frame));
	*/
	//static tf::TransformBroadcaster br;
	/*
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = p_frame;
	transformStamped.child_frame_id = c_frame;
	transformStamped.transform.translation.x = 0;
	transformStamped.transform.translation.y = 0;
	transformStamped.transform.translation.z = 0;
	*/

	/*
	setOrigin(tf::Vector3(px, py, pz));

	tf2::Quaternion q;
	q.setRPY(roll, pich, yaw);
	transformStamped.transform.rotation.x =  0;//q.x();
	transformStamped.transform.rotation.y =  0;//q.y();
	transformStamped.transform.rotation.z =  0;//q.z();
	transformStamped.transform.rotation.w =  1;//q.w();
	*/
	//br.sendTransform(transformStamped);

	/*	
	sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(info_mgr.getCameraInfo()));
	if (info->K[0] != 0.0 && 
	    (cv_img_depth.image.cols != info->width
	     || cv_img_depth.image.rows != info->height))
	  {
	    info.reset(new sensor_msgs::CameraInfo());
	  }
	if (info->K[0] == 0.0)
	  {
	    info->width = cv_img_depth.image.cols;
	    info->height = cv_img_depth.image.rows;
	  } 
	info->header.stamp = time;
	info->header.frame_id = c_frame;

	info_pub.publish( info );
	*/
      }
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinectv2client");
  Connection c;
  c.test();
  return 0;
}
