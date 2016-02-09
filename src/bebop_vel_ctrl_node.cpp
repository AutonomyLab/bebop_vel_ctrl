#include <ros/ros.h>

#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_vel_ctrl_node");

  ros::NodeHandle nh;

  ROS_INFO("Starting bebop_vel_ctrl_node ...");
  bebop_vel_ctrl::BebopVelCtrl vel_ctrl(nh);

  vel_ctrl.Spin();
  return 0;
}
