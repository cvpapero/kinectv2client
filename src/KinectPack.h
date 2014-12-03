#pragma once
// ver 1.1
#include <msgpack.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
class Image{
 public:
  int width;
  int height;
  int channels;
  std::vector<unsigned char> image;
  MSGPACK_DEFINE(
		 width,
		 height,
		 channels,
		 image
		 )
    };
class Color : public Image{
 public:
  MSGPACK_DEFINE(
		 width,
		 height,
		 channels,
		 image
		 );
};
class Depth : public Image{
 public:
  MSGPACK_DEFINE(
		 width,
		 height,
		 channels,
		 image
		 );
};
class BodyIndex : public Image{
 public:
  MSGPACK_DEFINE(
		 width,
		 height,
		 channels,
		 image
		 );
};
class LeanAmount{
 public:
  float x;
  float y;
  MSGPACK_DEFINE(
		 x,
		 y
		 );
};
class HandInfo{
 public:
  int handState;
  int trackingCOnfidence;
  MSGPACK_DEFINE(
		 handState,
		 trackingCOnfidence
		 );
};
class Position{
 public:
  float x;
  float y;
  float z;
  MSGPACK_DEFINE(
		 x,
		 y,
		 z
		 );
};
class Position2D{
 public:
  float x;
  float y;
  MSGPACK_DEFINE(
		 x,
		 y
		 );
};
class Orientation{
 public:
  float w;
  float x;
  float y;
  float z;
  MSGPACK_DEFINE(
		 w,
		 x,
		 y,
		 z
		 );
};
class JointInfo{
 public:
  int jointType;
  int trackingState;
  Position position;
  Position2D positionColorSpace;
  Orientation orientation;
  MSGPACK_DEFINE(
		 jointType,
		 trackingState,
		 position,
		 positionColorSpace,
		 orientation
		 );
};
class JointsInfo{
 public:
  std::vector< JointInfo > joints;
  int sizeJoint;
  MSGPACK_DEFINE(
		 joints,
		 sizeJoint
		 );
};
class BodyInfo{
 public:
  int bodyIndex;
  unsigned long long trackingId;
  bool isTracked;
  bool isRestricted;
  LeanAmount leanAmount;
  HandInfo rightHandState;
  HandInfo leftHandState;
  JointsInfo jointsInfo;
  MSGPACK_DEFINE(
		 bodyIndex,
		 trackingId,
		 isTracked,
		 isRestricted,
		 rightHandState,
		 leftHandState,
		 jointsInfo
		 );
};
class BodyInfoTest{
 public:
  std::string jsonBodyInfo;
  MSGPACK_DEFINE(
		 jsonBodyInfo
		 );
};
class KinectPack{
 public:
  Color imageColor;
  Depth imageDepth;
  BodyIndex imageBodyIndex;
  //std::vector< BodyInfo > bodies;
  BodyInfoTest bodies;
  MSGPACK_DEFINE(
		 imageColor,
		 imageDepth,
		 imageBodyIndex,
		 bodies
		 );
}; 
