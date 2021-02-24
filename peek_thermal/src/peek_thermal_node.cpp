#include <ros/ros.h>
#include "peek_thermal/PeekThermal.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "peek_thermal");
  ros::NodeHandle node;
  ros::NodeHandle nodeHandle("~");



  peek_thermal::PeekThermal peekThermal(node, nodeHandle);

  ros::spin();
  return 0;
}
