#ifndef BEBOP_VEL_CTRL
#define BEBOP_VEL_CTRL

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>

#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>

#include "bebop_vel_ctrl/bebop_vel_model.h"

namespace bebop_vel_ctrl
{

namespace util {

inline void ResetCmdVel(geometry_msgs::Twist& v)
{
  v.linear.x = 0.0;
  v.linear.y = 0.0;
  v.linear.z = 0.0;
  v.angular.x = 0.0;
  v.angular.y = 0.0;
  v.angular.z = 0.0;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val)
{
  if (nh.getParam(key, val))
  {
    ROS_INFO_STREAM("[VCTL] Param " << key << " : " << val);
    return true;
  }
  ROS_WARN_STREAM("[VCTL] Param " << key << " not found/set.");
  return false;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val, const T& default_val)
{
  nh.param(key, val, default_val);
  ROS_INFO_STREAM("[VCTL] Param " << key << " : " << val);
}

}

typedef message_filters::sync_policies::ApproximateTime<
    bebop_msgs::Ardrone3PilotingStateAltitudeChanged,
    bebop_msgs::Ardrone3PilotingStateAttitudeChanged,
    bebop_msgs::Ardrone3PilotingStateSpeedChanged> BebopSyncPolicy_t;

class BebopVelCtrl
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // This is required for proper initialization of PID dynamic reconfigure
  ros::NodeHandle nh_pid_vx;
  ros::NodeHandle nh_pid_vy;
  ros::NodeHandle nh_pid_yaw;
  ros::NodeHandle nh_pid_alt;

  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> bebop_alt_sub;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> bebop_att_sub;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateSpeedChanged> bebop_speed_sub;
  message_filters::Synchronizer<BebopSyncPolicy_t> bebop_sync;

  ros::Subscriber sub_setpoint_cmd_vel_;
  ros::Publisher pub_ctrl_cmd_vel_;

  geometry_msgs::Twist ctrl_twist_;

  // Bebop dynamic models for pitch->vx and roll->vy
  boost::shared_ptr<BebopVelocityTiltModel> velx_model; // pitch
  boost::shared_ptr<BebopVelocityTiltModel> vely_model; // roll

  // Bebop internal params (driver reads/sets them)
  bool beb_param_recv;
  double beb_maxtilt_rad;
  double beb_max_speed_vert_m;
  double beb_max_speed_rot_rad;

  // Params
  /* If any of these two params are set, the corresponding
   * degree of freedom is conrolled in positional space (not velocity space).
   * Input setpoint is treated as an absolute value
   * */
  bool abs_yaw_ctrl;
  bool abs_alt_ctrl;

  double time_delay_s;
  double Cx;
  double Cy;
  double update_freq;
  double max_linear_vel;  // m/s
  double max_angular_vel; // rad/s
  double max_vertical_vel; // m/s

  // Bebop Attitude States
  ros::Time bebop_recv_time;
  double beb_roll_rad;
  double beb_pitch_rad;
  double beb_yaw_rad;
  double beb_yaw_ref_rad;
  double beb_alt_m;

  // Bebop Velocities
  double beb_vx_m;
  double beb_vy_m;
  double beb_vz_m;
  double beb_vyaw_rad;

  // PID Controllers
  ros::Time pid_last_time;
  control_toolbox::Pid pid_vx;
  control_toolbox::Pid pid_vy;
  control_toolbox::Pid pid_yaw;
  control_toolbox::Pid pid_alt;

  // Internal
  double beb_vx_pred_m;
  double beb_vy_pred_m;

  void BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                         const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr,
                         const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr);

  // PID control happens here
  void SetpointCmdvelCallback(const geometry_msgs::TwistConstPtr& twist_ptr_);

public:
  BebopVelCtrl(ros::NodeHandle& nh);

  void Reset();
  virtual void Spin();
};

}  // namespace bebop_vel_ctrl

#endif  // BEBOP_VEL_CTRL
