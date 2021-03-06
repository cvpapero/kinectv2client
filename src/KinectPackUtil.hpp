#pragma
#include "KinectPack.h"
#include "opencv2/opencv.hpp"
#include "picojson.h"
#include <sstream>
#include <iostream>
#include "Util.h"
namespace KinectPackUtil{

  void convertImageToMat( const Image& src, cv::Mat& dst ){
    dst = Util::convertVec2Mat( src.image );
  }

  void convertDepth16UToDepth8U( const cv::Mat& src, cv::Mat& dst ){
    dst = cv::Mat::zeros( src.rows, src.cols, CV_8UC1 );
    for( int row = 0; row < src.rows ; ++row )
      {
	for( int col = 0; col < src.cols ; ++col )
	  {
	    dst.at< unsigned char >( row, col ) = src.at< short >( row, col ) / 8000. * 255.;
	  }
      }
  }

  void drawSkeleton( cv::Mat &dst, const KinectPack kinectPack, double cols, double rows, const int radius = 10, const int thickness = 2 )
  {
    picojson::value v;
    std::string err;
    picojson::parse( v, kinectPack.bodies.jsonBodyInfo.begin(), kinectPack.bodies.jsonBodyInfo.end(), &err );
    if ( err.empty() )
      {
	picojson::object &objBodyInfo = v.get< picojson::object >();
	std::string verstion = objBodyInfo["version"].get< std::string >();
	picojson::array arrayBody = objBodyInfo["bodies"].get< picojson::array >();
	for ( std::vector<picojson::value>::iterator itrBody = arrayBody.begin() ; itrBody != arrayBody.end() ; ++itrBody )
	  {
	    picojson::object &objBody = itrBody->get< picojson::object >();
	    if ( objBody["isTracked"].get< bool >() )
	      {
		picojson::array arrayJoint = objBody["Joints"].get< picojson::array >();
		for( std::vector<picojson::value>::iterator itrJoint = arrayJoint.begin() ; itrJoint != arrayJoint.end() ; ++itrJoint )
		  {
		    picojson::object &objJoint = itrJoint->get< picojson::object >();
		    picojson::object &objPositionColorSpace = objJoint["PositionColorSpace"].get< picojson::object >();
		    cv::circle( dst, cv::Point( (int)objPositionColorSpace["X"].get<double>()*cols, (int)objPositionColorSpace["Y"].get<double>()*rows ), radius, cv::Scalar( 0, 200, 200 ), thickness );
		  }
	      }
	  }
      }
    else
      {
	std::cerr << __FUNCTION__ << ":" << err << std::endl;
      }
  }	
} 
