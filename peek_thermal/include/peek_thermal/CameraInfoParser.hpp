#pragma once

#include <sensor_msgs/CameraInfo.h>
#include "opencv2/core/core.hpp"




namespace peek_thermal {

/*!
 * Class containing the CameraInfoParseric part of the package.
 */
class CameraInfoParser
{
 public:
  /*!
   * Constructor.
   */
  CameraInfoParser();

  //! Structure for camera parameters
  struct camera_params_{cv::Mat proj_;
                        cv::Mat cam_;
                        cv::Mat dist_;
                        u_int width_;
                       u_int height_;};

  //! Camera Info recieved
  bool cameraInfoRecieved_;

  /*!
   * Destructor.
   */
  virtual ~CameraInfoParser();

  /*!
   * Parsing CamInfo ROS msg
   * into OpenCV Mat.
   * @param data the new data.
   */
  struct camera_params_ parseCamInfoMsg(
              const sensor_msgs::CameraInfo::ConstPtr &info_msg);

 private:

  //! Camera Parameters
  camera_params_ cam_params_;


};

} /* namespace */
