#include "peek_thermal/PeekThermal.hpp"


// STD
#include <string>

namespace peek_thermal {

PeekThermal::PeekThermal(ros::NodeHandle node,
                                     ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  img_sub_ = nodeHandle_.subscribe(imgTopic_, 1,
                                      &PeekThermal::imgCallback, this);
  cam_info_sub_ = nodeHandle_.subscribe(camInfoTopic_, 1,
                                      &PeekThermal::camInfoCallback, this);

  //dynamic reconfigure
  f_ = boost::bind(&PeekThermal::parameterCallback, this, _1, _2) ;
  server_.setCallback(f_);

  ROS_INFO("Successfully launched node.");
}

PeekThermal::~PeekThermal()
{
}

void PeekThermal::parameterCallback(peek_thermal::hsv_parametersConfig &config, uint32_t level){

  lower_H_ = unsigned(config.lower_H);
  lower_S_ = unsigned(config.lower_S);
  lower_V_ = unsigned(config.lower_V);
  upper_H_ = unsigned(config.upper_H);
  upper_S_ = unsigned(config.upper_S);
  upper_V_ = unsigned(config.upper_V);
}


bool PeekThermal::readParameters(){

  if (!nodeHandle_.getParam("img_topic", imgTopic_)) return false;
  if (!nodeHandle_.getParam("cam_info_topic", camInfoTopic_)) return false;
  if (!nodeHandle_.getParam("visualization", visualization_)) return false;
  return true;
}


void PeekThermal::imgCallback(const sensor_msgs::Image::ConstPtr &img){

  //check if camera info was correctly recieved
//  if(!cameraInfoParser_.cameraInfoRecieved_){
//    ROS_WARN("Waiting for Camera Info!");
//    return;
//  }

  //convert ROS image to CV image
  cv::Mat ori_img;
  ori_img = convertToCVimage_.convertROStoCV(img);

  if(visualization_)
    showInOpenCVWindow(ori_img, "_ORIGINAL");
    
 //Convert to Thermal data  -------- Added by CARG
 cv::Mat thermal_img;
 thermal_img = ori_img[]/10.0 - 100   
 // --------------------------------------

  //process image
  cv::Mat hsv_filtered_img;
  hsv_filtered_img = filteringHSV(ori_img);

  cv::Mat grayscale_img;
  grayscale_img = convertToGrayscale(ori_img);
}


void PeekThermal::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info){

  cameraParams_ = cameraInfoParser_.parseCamInfoMsg(cam_info);
  ROS_INFO("Camera Info recieved!");
  cam_info_sub_.shutdown();
  ros::Duration(1.0).sleep();
}


cv::Mat PeekThermal::filteringHSV(cv::Mat &img){

  cv::Mat hsv_img, filtered_img;
  cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

  try
  {
    inRange(hsv_img, cv::Scalar(lower_H_, lower_S_, lower_V_),
            cv::Scalar(upper_H_, upper_S_, upper_V_), filtered_img);

    if(visualization_)
      showInOpenCVWindow(filtered_img, "HSV");
  }
  catch( cv::Exception& e ){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  return filtered_img;
}


cv::Mat PeekThermal::convertToGrayscale(cv::Mat &img){

  cv::Mat grayscale_img;
  cv::cvtColor(img, grayscale_img,cv::COLOR_BGR2GRAY);

  if(visualization_)
    showInOpenCVWindow(grayscale_img, "_GRAYSCALE");

  return grayscale_img;
}

//--------------- Added by CARG -------
cv::Mat PeekThermal::convertToThermal(cv::Mat &img){
  
  cv::Mat thermal_img;
  cv::cvtColor(img, thermal_img, cv::COLOR_BGR2RGB);
  
  if (visualization_)
    showInOpenCVWindow(thermal_img, "_THERMAL");
    
  return thermal_img;  
}
//--------------------------------

void PeekThermal::showInOpenCVWindow(cv::Mat &img,
                                           std::string window_name){

  cv::namedWindow(ros::this_node::getName()+window_name,
                  cv::WINDOW_NORMAL);
  imshow( ros::this_node::getName()+window_name, img );
  cv::resizeWindow(ros::this_node::getName()+window_name,
                   int(cameraParams_.width_/2),
                   int(cameraParams_.height_/2));
  cv::waitKey(3);
}


} /* namespace */
