#include "peek_thermal/CameraInfoParser.hpp"

namespace peek_thermal {

CameraInfoParser::CameraInfoParser()
    : cameraInfoRecieved_(false)
{
}

CameraInfoParser::~CameraInfoParser()
{

}



CameraInfoParser::camera_params_
        CameraInfoParser::parseCamInfoMsg
                (const sensor_msgs::CameraInfo::ConstPtr &info_msg)
{
  CameraInfoParser::camera_params_ st_par;
  //projection matrix
  st_par.proj_ = cv::Mat(3,4,CV_64F);
  {
      double* row_0 = st_par.proj_.ptr<double>(0);
      row_0[0] = info_msg.get()->P[0]; row_0[1] = info_msg.get()->P[1];
      row_0[2] = info_msg.get()->P[2]; row_0[3]= info_msg.get()->P[3];
      double* row_1 = st_par.proj_.ptr<double>(1);
      row_1[0] = info_msg.get()->P[4]; row_1[1] = info_msg.get()->P[5];
      row_1[2] = info_msg.get()->P[6]; row_1[3]= info_msg.get()->P[7];
      double* row_2 = st_par.proj_.ptr<double>(2);
      row_2[0] = info_msg.get()->P[8]; row_2[1] = info_msg.get()->P[9];
      row_2[2] = info_msg.get()->P[10]; row_2[3]= info_msg.get()->P[11];
  }

 //camera matrix
 st_par.cam_ = cv::Mat(3,3,CV_64F);{
      double* row_0 = st_par.cam_.ptr<double>(0);
      row_0[0] = info_msg.get()->K[0]; row_0[1] = info_msg.get()->K[1];
      row_0[2] = info_msg.get()->K[2];
      double* row_1 = st_par.cam_.ptr<double>(1);
      row_1[0] = info_msg.get()->K[3]; row_1[1] = info_msg.get()->K[4];
      row_1[2] = info_msg.get()->K[5];
      double* row_2 = st_par.cam_.ptr<double>(2);
      row_2[0] = info_msg.get()->K[6]; row_2[1] = info_msg.get()->K[7];
      row_2[2] = info_msg.get()->K[8];
 }

 //distortion coefficients
 st_par.dist_ = cv::Mat(1,5,CV_64F);{
      double* row_0 = st_par.dist_.ptr<double>(0);
      row_0[0] = info_msg.get()->D[0]; row_0[1] =  info_msg.get()->D[1];
      row_0[2] = info_msg.get()->D[2]; row_0[3] = info_msg.get()->D[3];
      row_0[4] = info_msg.get()->D[4];
 }

 //resolution
 st_par.width_ = info_msg.get()->width;
 st_par.height_= info_msg.get()->height;

 cameraInfoRecieved_ = true;

 return st_par;
}



} /* namespace */
