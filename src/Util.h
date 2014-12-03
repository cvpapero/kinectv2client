#pragma once
#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ctime>
namespace Util{
  std::string toString(int n);
  void replaceAll(std::string &str,const std::string from, const std:: string to);
  std::vector<std::string> split(std::string s, std::string t);
  std::vector<std::string> splitFirst(std::string s, const std::string t) ;
  cv::Mat convertVec2Mat( const std::vector<unsigned char>& vec );
  cv::Mat convertString2Mat(std::string str);
  std::string convertMat2String(cv::Mat mat);
  //std::string getCurrentTimeString();
} 
