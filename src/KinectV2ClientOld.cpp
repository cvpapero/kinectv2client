/*
2014.10.29----------------------------
あたらしく作られたKinectV2Serverに対応する
JSON
それから、JSON→ROSメッセージ変換ヘッダファイルを作成する

  かれんとにconfigKinectServer.txtが必要

*/

 
#include "Connectioni.h"
#include <fstream>
#include <vector>
#include "Util.h"

//#include "Util.h"
#include "KinectPack.h"
#include "zmq.hpp"
#include <sstream>
#include <iostream>
#include "Connectioni.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MessagePackUtility.hpp"
#include "KinectPackUtil.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <humans_msgs/Humans.h>

#include "JsonToMsg.hpp"

//typedef std::map<std::string, JointInfo> JointMap;
 
using namespace std;

Connection::Connection(){
  initialize();
}

Connection::Connection( const int threads ){
  
}

void Connection::initialize(){
  cout << __FUNCTION__ << endl;
  try
    {
      zmq::context_t *conetext = new zmq::context_t( 10 );
      socket = new zmq::socket_t( *conetext, ZMQ_REQ );
      
      ifstream ifs( "//home//papero//catkin_ws//src//kinectv2client//src//configKinectServer.txt" );
      string line;
      string ipaddress;
      string port;
      while( getline( ifs, line ) )
	{
	  vector< string > result = Util::split( line, ":" );
	  if( result.size() != 2 )
	    {
	      cout << "file format error" << endl;				
	    }
	  ipaddress = result[ 0 ];
	  port = result[ 1 ];
	}
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

void Connection::start(){
  
  
}
void Connection::send( const string message ){
  zmq::message_t message_send( message.size() );
  memcpy( message_send.data(), message.data(), message.size() );
  socket->send( message_send );
}
void Connection::recv( string &message ){
  zmq::message_t message_recv;
  socket->recv( &message_recv );
  message = std::string(static_cast<char*>(message_recv.data()), message_recv.size());
}


void test(){

  // サーバと通信するために必要
  // 接続先情報を読み込んでコネクションを張る等の初期化も同時に行われる.
  Connection connection; 
  
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher color_pub = it.advertise("/camera/image/color", 1);
  //image_transport::Publisher depth_pub = it.advertise("/camera/image/depth", 1);
  //image_transport::Publisher bodyindex_pub = it.advertise("/camera/image/bodyindex", 1);
  ros::Publisher kinectv2_pub = nh.advertise<humans_msgs::Humans>("/humans/KinectV2",10);

  while( true ){

    // REP-REQ形式なので要求メッセージを送信する
    connection.send("test message"); // 現在はどんな文字列を送っても違いはない
    
    // データを受信する
    string message;
    connection.recv( message ); // サーバから帰ってくる文字列
    
    // メッセージを指定形式(今回はKinectPack)に変換
    // 送られてくるデータ形式と異なるとエラー
    KinectPack kinectPack;
    MsgPackUtil::unpack( message, kinectPack);
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
    //cout << "cols:" <<cols_scale << endl;
    //cout << "rows:" <<rows_scale << endl;
    JsonToMsg::body(kinectPack, &kinect_msg, cols_scale, rows_scale);
    // 検知した人間のJoint情報を元にスケルトンを描画
    //KinectPackUtil::drawSkeleton( imageColor, kinectPack, cols_scale, rows_scale); 
    // 表示
    /* 
    try
      {
	//cv::resize(imageColor, imageColor, cv::Size(960,540));
	cv::imshow("color", imageColor);
      }
    catch ( const cv::Exception &e )
      {
	cout << "imshow(Color): " << e.what() << endl;
      }
    */
    /*
    try
      {
	cv::imshow("depth", imageDepth);
      }
    catch ( const cv::Exception &e )
      {
	cout << "imshow(Depth): " << e.what() << endl;
      }
    //cv::imshow("depth8U", imageDepth8U );
    */
    try
      {
	cv::imshow("bodyIndex", imageBodyIndex);
      }
    catch ( const cv::Exception &e )
      {
	cout << "imshow(BodyIndex): " << e.what() << endl;
	cout << "(" << imageBodyIndex.cols << "," << imageBodyIndex.rows << ")" << endl;
      }
    cv::waitKey(1);
    
    cv_bridge::CvImage cv_img_color;//, cv_img_depth, cv_img_bodyindex; 
    cv_img_color.header.stamp = ros::Time::now(); 
    cv_img_color.header.frame_id = "camera_frame"; 
    cv_img_color.encoding = "bgr8"; 
    cv_img_color.image = imageColor; 
    /*
    cv_img_depth.header.stamp = ros::Time::now(); 
    cv_img_depth.header.frame_id = "camera_frame"; 
    cv_img_depth.encoding = "mono16";  
    cv_img_depth.image = imageDepth; 

    cv_img_bodyindex.header.stamp = ros::Time::now(); 
    cv_img_bodyindex.header.frame_id = "camera_frame"; 
    cv_img_bodyindex.encoding = "mono8";  
    cv_img_bodyindex.image = imageBodyIndex; 
    */
    color_pub.publish(cv_img_color.toImageMsg());
    /*
    depth_pub.publish(cv_img_depth.toImageMsg());
    bodyindex_pub.publish(cv_img_bodyindex.toImageMsg());
    */
   
    kinect_msg.header.stamp = ros::Time::now();
    kinect_msg.header.frame_id ="camera_link";
    //kinect_msg.image = *cv_img_color.toImageMsg();
    kinectv2_pub.publish(kinect_msg);
   
    
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "kinectv2client");
  test();
  return 0;
}
