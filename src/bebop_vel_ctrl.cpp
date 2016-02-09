#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

namespace bebop_vel_ctrl
{

BebopVelCtrl::BebopVelCtrl(ros::NodeHandle &nh)
  : nh_(nh),
    nh_priv_("~"),
    nh_pid_vx_(nh_, "pid_forward"),
    nh_pid_vy_(nh_, "pid_lateral"),
    nh_pid_yaw_(nh_, "pid_yaw"),
    nh_pid_alt_(nh_, "pid_alt"),
    sub_setpoint_cmd_vel_(nh_.subscribe("setpoint/cmd_vel", 1, &BebopVelCtrl::SetpointCmdvelCallback, this)),
    pub_ctrl_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1)),
    sub_bebop_alt_(nh_, "bebop/states/ARDrone3/PilotingState/AltitudeChanged", 1),
    sub_bebop_att_(nh_, "bebop/states/ARDrone3/PilotingState/AttitudeChanged", 1),
    sub_bebop_speed_(nh_, "bebop/states/ARDrone3/PilotingState/SpeedChanged", 1),
    subsync_bebop_(BebopSyncPolicy_t(10), sub_bebop_alt_, sub_bebop_att_, sub_bebop_speed_),
    beb_param_recv(false),
    bebop_recv_time_(0),
    pid_vx_(new control_toolbox::Pid()),
    pid_vy_(new control_toolbox::Pid()),
    pid_yaw_(new control_toolbox::Pid()),
    pid_alt_(new control_toolbox::Pid()),
    beb_vx_pred_m_(0.0),
    beb_vy_pred_m_(0.0)
{
  util::GetParam(nh_priv_, "abs_yaw_ctrl", param_abs_yaw_ctrl_, false);
  util::GetParam(nh_priv_, "abs_alt_ctrl", param_abs_yaw_ctrl_, false);

  util::GetParam(nh_priv_, "model_delay_s", param_time_delay_, 0.262137);
  util::GetParam(nh_priv_, "model_cx", param_model_cx_, -0.576335778073963);
  util::GetParam(nh_priv_, "model_cy", param_model_cy_, -0.584975281133000);
  util::GetParam(nh_priv_, "update_freq", param_update_freq_, 30.0);

  // We initialize PIDs through its nodehandle constructor,
  // The following will set some default values for the parameters if the user
  // does not specify them. This plays nice with Dynamic Reconfigure
  nh_pid_vx_.setParam("p", nh_pid_vx_.param("p", 0.15));
  nh_pid_vx_.setParam("i", nh_pid_vx_.param("i", 0.4));
  nh_pid_vx_.setParam("d", nh_pid_vx_.param("d", 0.0));
  nh_pid_vx_.setParam("i_clamp", nh_pid_vx_.param("i_clamp", 0.02));

  nh_pid_vy_.setParam("p", nh_pid_vy_.param("p", 0.15));
  nh_pid_vy_.setParam("i", nh_pid_vy_.param("i", 0.4));
  nh_pid_vy_.setParam("d", nh_pid_vy_.param("d", 0.0));
  nh_pid_vy_.setParam("i_clamp", nh_pid_vy_.param("i_clamp", 0.02));

  nh_pid_yaw_.setParam("p", nh_pid_yaw_.param("p", 0.5));
  nh_pid_yaw_.setParam("i", nh_pid_yaw_.param("i", 0.0));
  nh_pid_yaw_.setParam("d", nh_pid_yaw_.param("d", 0.0));
  nh_pid_yaw_.setParam("i_clamp", nh_pid_yaw_.param("i_clamp", 0.02));

  nh_pid_alt_.setParam("p", nh_pid_alt_.param("p", 0.5));
  nh_pid_alt_.setParam("i", nh_pid_alt_.param("i", 0.0));
  nh_pid_alt_.setParam("d", nh_pid_alt_.param("d", 0.0));
  nh_pid_alt_.setParam("i_clamp", nh_pid_alt_.param("i_clamp", 0.02));

  ROS_ASSERT(pid_vx_ && pid_vy_ && pid_yaw_ && pid_alt_);

  pid_vx_->init(nh_pid_vx_);
  pid_vy_->init(nh_pid_vy_);
  pid_yaw_->init(nh_pid_yaw_);
  pid_alt_->init(nh_pid_alt_);

  model_velx_ = boost::make_shared<bebop_vel_ctrl::BebopVelocityTiltModel>(param_model_cx_, GRAV_CST);
  model_vely_ = boost::make_shared<bebop_vel_ctrl::BebopVelocityTiltModel>(param_model_cy_, -GRAV_CST);

  subsync_bebop_.registerCallback(boost::bind(&BebopVelCtrl::BebopSyncCallback, this, _1, _2, _3));
}

void BebopVelCtrl::BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                                     const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr,
                                     const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr)
{
  bebop_recv_time_ = ros::Time::now();

  if (!beb_param_recv)
  {
    // This is sketchy, I need to find a way to get these params
    if (!util::GetParam(nh_, "/bebop/bebop_nodelet/PilotingSettingsMaxTiltCurrent", beb_maxtilt_rad))
    {
      return;
    }
    beb_maxtilt_rad = angles::from_degrees(beb_maxtilt_rad);
    if (!util::GetParam(nh_, "/bebop/bebop_nodelet/SpeedSettingsMaxVerticalSpeedCurrent", beb_max_speed_vert_m))
    {
      return;
    }
    if (!util::GetParam(nh_, "/bebop/bebop_nodelet/SpeedSettingsMaxRotationSpeedCurrent", beb_max_speed_rot_rad))
    {
      return;
    }
    beb_max_speed_rot_rad = angles::from_degrees(beb_max_speed_rot_rad);

    // use first yaw as ref point
    beb_yaw_ref_rad_ = -att_ptr->yaw;
    beb_param_recv = true;
  }

  beb_roll_rad_ = att_ptr->roll;
  beb_pitch_rad_ = -att_ptr->pitch;
  beb_yaw_rad_ = -att_ptr->yaw;
  beb_alt_m_ = alt_ptr->altitude;
  ROS_INFO_STREAM("Current Bebop State: RPY & ALT: "
                  << angles::to_degrees(beb_roll_rad_) << " "
                  << angles::to_degrees(beb_pitch_rad_) << " "
                  << angles::to_degrees(beb_yaw_rad_) << " "
                  << beb_alt_m_ << " ");

  // Bebop Speed is in world coordinates (ESD coords (we convert it to ENU))
  // yaw is already converted (with respect to magnetic north)
  const double vx_enu = speed_ptr->speedX;
  const double vy_enu = -speed_ptr->speedY;
  const double vz_enu = -speed_ptr->speedZ;

  beb_vx_m_ = cos(beb_yaw_rad_) * vx_enu + sin(beb_yaw_rad_) * vy_enu;
  beb_vy_m_ = -sin(beb_yaw_rad_) * vx_enu + cos(beb_yaw_rad_) * vy_enu;

  ROS_INFO_STREAM("Current Bebop Velcoities: XYZ: "
                  << beb_vx_m_ << " "
                  << beb_vy_m_ << " "
                  << beb_vz_m_ << " ");
  beb_vyaw_rad_ = 0.0; // this is unknown


  model_velx_->Simulate(param_time_delay_, 0.05, beb_vx_m_, beb_pitch_rad_);
  model_vely_->Simulate(param_time_delay_, 0.05, beb_vy_m_, beb_roll_rad_);

  ROS_INFO_STREAM("Simulated Bebop Velocities: XY: "
                  << model_velx_->GetVel() << " " << model_vely_->GetVel());
  ROS_WARN_STREAM("Last predicted bebop vels: XY:"
                  << beb_vx_pred_m_ << " " << beb_vy_pred_m_);

  beb_vx_pred_m_ = beb_vx_m_;
  beb_vy_pred_m_ = beb_vy_m_;
}

void BebopVelCtrl::SetpointCmdvelCallback(const geometry_msgs::TwistConstPtr &twist_ptr_)
{
  if (!beb_param_recv)
  {
    ROS_WARN("[VCTL] Bebop params are not set!");
    return;
  }

  // This condition is already checked by the main loop for safety
  const ros::Time& t_now = ros::Time::now();
  if ((t_now - bebop_recv_time_).toSec() > 1.0) return;

  // CLAMP Input Setpoints
  geometry_msgs::Twist setpoint = *twist_ptr_;
  CLAMP(setpoint.linear.x, -param_max_linear_vel_, param_max_linear_vel_);
  CLAMP(setpoint.linear.y, -param_max_linear_vel_, param_max_linear_vel_);
  CLAMP(setpoint.linear.z, -param_max_vertical_vel_, param_max_vertical_vel_);
  CLAMP(setpoint.angular.z, -param_max_angular_vel_, param_max_angular_vel_);

  // PID Control Loop
  const ros::Duration& dt = t_now - pid_last_time_;
  const double pitch_ref = pid_vx_->computeCommand(setpoint.linear.x - model_velx_->GetVel(), dt);
  const double roll_ref  = pid_vy_->computeCommand(setpoint.linear.y - model_vely_->GetVel(), dt);

  // If abs yaw ctrl is set, setpoint.angular.z is an angle
  const double vyaw_ref = (param_abs_yaw_ctrl_) ?
        pid_yaw_->computeCommand(angles::normalize_angle(setpoint.angular.z - beb_yaw_rad_), dt) :
        setpoint.angular.z;

  // If abs alt ctrl is set, setpoint.linear.z is an altitude
  const double vz_ref = (param_abs_alt_ctrl_) ?
        pid_alt_->computeCommand(setpoint.linear.z - beb_alt_m_, dt) :
        setpoint.linear.z;

  // Convert PID output  into normalized cmd_vel (-1 -> 1)
  util::ResetCmdVel(ctrl_twist_);
  ctrl_twist_.linear.x =  pitch_ref / beb_maxtilt_rad;
  ctrl_twist_.linear.y =  roll_ref / beb_maxtilt_rad;
  ctrl_twist_.angular.z = vyaw_ref / beb_max_speed_rot_rad;
  ctrl_twist_.linear.z = vz_ref / beb_max_speed_vert_m;

  // CLAMP and filter output
  CLAMP(ctrl_twist_.linear.x, -1.0, 1.0);
  CLAMP(ctrl_twist_.linear.y, -1.0, 1.0);
  CLAMP(ctrl_twist_.linear.z, -1.0, 1.0);
  CLAMP(ctrl_twist_.angular.z, -1.0, 1.0);
  FILTER_SMALL_VALS(ctrl_twist_.linear.x, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.y, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.z, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.angular.z, 0.01);

  // Simulate Bebop's velocity given the reference tilt to update (predict) feedback
  // this to compensate for low frequency velocity feedback
  // We again convert from normalized values to angles to take into effect any clamp/filtering effects
  model_velx_->Reset(model_velx_->GetVel(), CLAMP(ctrl_twist_.linear.x * beb_maxtilt_rad, -beb_maxtilt_rad, beb_maxtilt_rad));
  model_velx_->Simulate(dt.toSec());

  model_vely_->Reset(model_vely_->GetVel(), CLAMP(ctrl_twist_.linear.y * beb_maxtilt_rad, -beb_maxtilt_rad, beb_maxtilt_rad));
  model_vely_->Simulate(dt.toSec());

  pid_last_time_ = ros::Time::now();

  pub_ctrl_cmd_vel_.publish(ctrl_twist_);

  ROS_ERROR_STREAM("[VCTL] CMD_VEL for: " << ctrl_twist_.linear.x <<
                   " left: " << ctrl_twist_.linear.y <<
                   " up: " << ctrl_twist_.linear.z <<
                   " cw: " << ctrl_twist_.angular.z);
}

void BebopVelCtrl::Reset()
{
  util::ResetCmdVel(ctrl_twist_);
  pub_ctrl_cmd_vel_.publish(ctrl_twist_);
}

void BebopVelCtrl::Spin()
{
  ROS_INFO("[VCTL] Spinnig");

  // Safety
  Reset();

  ros::Rate loop_rate(param_update_freq_);

  pid_last_time_ = ros::Time::now();
  while (ros::ok())
  {
    try
    {
      bool do_reset = false;
      if ((ros::Time::now() - bebop_recv_time_).toSec() > 1.0)
      {
        ROS_WARN("[VCTL] Bebop state feedback is older than 1 second! Resetting.");
        do_reset = true;
      }

      if ((ros::Time::now() - pid_last_time_).toSec() > (5.0 / param_update_freq_))
      {
        ROS_WARN("[VCTL] Input ctrl_cmd_vel is old or slow! Resetting.");
        do_reset = true;
      }

      if (do_reset) Reset();

      ros::spinOnce();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[VCTL] Missed loop update rate of " << param_update_freq_);
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[VCTL] Runtime Exception: " << e.what());
      Reset();
    }
  }
}
}  // namespace bebop_vel_ctrl
