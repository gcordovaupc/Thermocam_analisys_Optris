#include "peek_thermal/ConvertToCVimage.hpp"

namespace peek_thermal {

ConvertToCVimage::ConvertToCVimage()
{
}

ConvertToCVimage::~ConvertToCVimage()
{

}


cv::Mat ConvertToCVimage::convertROStoCV(const sensor_msgs::Image::ConstPtr &img_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cv_ptr->image;
  }

  return cv_ptr->image;
}


} /* namespace */
