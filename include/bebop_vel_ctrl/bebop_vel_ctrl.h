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

#include "bebop_vel_ctrl/Debug.h"
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

}  // namespace util

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
  ros::NodeHandle nh_pid_vx_;
  ros::NodeHandle nh_pid_vy_;
  ros::NodeHandle nh_pid_yaw_;
  ros::NodeHandle nh_pid_alt_;

  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> sub_bebop_alt_;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> sub_bebop_att_;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateSpeedChanged> sub_bebop_speed_;
  message_filters::Synchronizer<BebopSyncPolicy_t> subsync_bebop_;

  ros::Subscriber sub_setpoint_cmd_vel_;
  ros::Publisher pub_ctrl_cmd_vel_;
  ros::Publisher pub_debug_;

  geometry_msgs::Twist setpoint_cmd_vel;
  geometry_msgs::Twist ctrl_twist_;

  // Bebop dynamic models for pitch->vx and roll->vy
  boost::shared_ptr<BebopVelocityTiltModel> model_velx_; // pitch
  boost::shared_ptr<BebopVelocityTiltModel> model_vely_; // roll

  // Bebop internal params (driver reads/sets them)
  bool beb_param_recv;
  double beb_maxtilt_rad_;
  double beb_max_speed_vert_m_;
  double beb_max_speed_rot_rad_;

  // Params
  /* If any of these two params are set, the corresponding
   * degree of freedom is conrolled in positional space (not velocity space).
   * Input setpoint is treated as an absolute value
   * */
  bool param_abs_yaw_ctrl_;
  bool param_abs_alt_ctrl_;

  double param_time_delay_;
  double param_model_cx_;
  double param_model_cy_;
  double param_update_freq_;
  bool param_safety_send_zero_;
  bool param_zero_xy_hover;

  double param_max_linear_vel_;  // m/s
  double param_min_alt_;
  double param_max_alt_;
  double param_feedback_pred_factor_;
  double param_delay_compensation_factor_;

  // Bebop Attitude States
  ros::Time bebop_recv_time_;
  double beb_roll_rad_;
  double beb_pitch_rad_;
  double beb_yaw_rad_;
  double beb_alt_m_;

  // Bebop Velocities
  double beb_vx_m_;
  double beb_vy_m_;
  double beb_vz_m_;
  double beb_vyaw_rad_;

  // PID Controllers
  ros::Time setpoint_recv_time_;
  ros::Time pid_last_time_;
  boost::shared_ptr<control_toolbox::Pid> pid_vx_;
  boost::shared_ptr<control_toolbox::Pid> pid_vy_;
  boost::shared_ptr<control_toolbox::Pid> pid_yaw_;
  boost::shared_ptr<control_toolbox::Pid> pid_alt_;

  bebop_vel_ctrl::Debug msg_debug_;

  // Internal
  double beb_vx_pred_m_;
  double beb_vy_pred_m_;

  void BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                         const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr,
                         const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr);

  // PID control happens here
  void SetpointCmdvelCallback(const geometry_msgs::TwistConstPtr& twist_ptr_);

  bool Update();

public:
  BebopVelCtrl(ros::NodeHandle& nh);

  void Reset();
  virtual void Spin();
};

}  // namespace bebop_vel_ctrl

#endif  // BEBOP_VEL_CTRL
