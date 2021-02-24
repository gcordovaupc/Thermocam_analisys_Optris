#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


namespace peek_thermal {

/*!
 * Class containing the ConvertToCVimageic part of the package.
 */
class ConvertToCVimage
{
 public:
  /*!
   * Constructor.
   */
  ConvertToCVimage();



  /*!
   * Destructor.
   */
  virtual ~ConvertToCVimage();

  /*!
   * Converting ROS Image
   * to opencv image
   * @param data ROS image.
   */
  cv::Mat convertROStoCV(
              const sensor_msgs::Image::ConstPtr &img_msg);

 private:




};

} /* namespace */
