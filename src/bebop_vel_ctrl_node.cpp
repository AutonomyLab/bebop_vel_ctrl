#include <ros/ros.h>

#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_vel_ctrl_node");

  ros::NodeHandle nh;

  ros::spin();
  return 0;
}
