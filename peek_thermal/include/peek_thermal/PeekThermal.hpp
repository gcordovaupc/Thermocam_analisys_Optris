#pragma once

#include "peek_thermal/CameraInfoParser.hpp"
#include "peek_thermal/ConvertToCVimage.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Trigger.h>
#include <dynamic_reconfigure/server.h>


#include <opencv2/highgui/highgui.hpp>

#include "peek_thermal/hsv_parametersConfig.h"





namespace peek_thermal {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class PeekThermal
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PeekThermal(ros::NodeHandle node, ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PeekThermal();

 private:


  /*!
   * \brief dynamic reconfigure
   */
  void parameterCallback(peek_thermal::hsv_parametersConfig&, uint32_t);

  dynamic_reconfigure::Server<peek_thermal::hsv_parametersConfig> server_;
  dynamic_reconfigure::Server<peek_thermal::hsv_parametersConfig>::CallbackType f_;
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void imgCallback(const sensor_msgs::Image::ConstPtr &img);

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

  /*!
   * \brief filters an img by HSV values
   * \param img input image
   * \return HSV filtered image
   */
  cv::Mat filteringHSV(cv::Mat &img);

  /*!
   * \brief Convertan img into grayscale format
   * \param img
   * \return grayscale img
   */
  cv::Mat convertToGrayscale(cv::Mat &img);


  /*!
   * Show opencv img in an opencv window
   * \param img opencv image
   * \param window_name name of the window
   */
  void showInOpenCVWindow(cv::Mat &img, std::string window_name);


  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber img_sub_;

  //! ROS topic subscriber.
  ros::Subscriber cam_info_sub_;

  ///Parameter

  //! ROS topic name to subscribe to.
  std::string imgTopic_;
  //! ROS topic name to subscribe to.
  std::string camInfoTopic_;
  //! Determine if vislualization should be on or off
  bool visualization_;

  //! CameraInfoParser computation object.
  CameraInfoParser cameraInfoParser_;
  CameraInfoParser::camera_params_ cameraParams_;

  //! ConvertToOpenCV computation object
  ConvertToCVimage convertToCVimage_;

  //#### HSV Filter #####
  //!lower_H
  u_int lower_H_;
  //!lower_S
  u_int lower_S_;
  //!lower_V
  u_int lower_V_;

  //!upper_H
  u_int upper_H_;
  //!upper_S
  u_int upper_S_;
  //!upper_V
  u_int upper_V_;
};

} /* namespace */
