#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

#include "bebop_vel_ctrl/bebop_vel_ctrl.h"

namespace bebop_vel_ctrl
{

BebopVelCtrl::BebopVelCtrl(ros::NodeHandle &nh)
  : nh_(nh),
    nh_priv_("~"),
    nh_pid_vx(nh_, "pid_forward"),
    nh_pid_vy(nh_, "pid_lateral"),
    nh_pid_yaw(nh_, "pid_yaw"),
    nh_pid_alt(nh_, "pid_alt"),
    sub_setpoint_cmd_vel_(nh_.subscribe("setpoint_cmd_vel", 1, &BebopVelCtrl::SetpointCmdvelCallback, this)),
    pub_ctrl_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("ctrl_cmd_vel", 1)),
    bebop_alt_sub(nh_, "bebop/states/ARDrone3/PilotingState/AltitudeChanged", 1),
    bebop_att_sub(nh_, "bebop/states/ARDrone3/PilotingState/AttitudeChanged", 1),
    bebop_speed_sub(nh_, "bebop/states/ARDrone3/PilotingState/SpeedChanged", 1),
    bebop_sync(BebopSyncPolicy_t(10), bebop_alt_sub, bebop_att_sub, bebop_speed_sub),
    beb_param_recv(false),
    bebop_recv_time(0),
    pid_vx(new control_toolbox::Pid()),
    pid_vy(new control_toolbox::Pid()),
    pid_yaw(new control_toolbox::Pid()),
    pid_alt(new control_toolbox::Pid()),
    beb_vx_pred_m(0.0),
    beb_vy_pred_m(0.0)
{
  util::GetParam(nh_priv_, "abs_yaw_ctrl", abs_yaw_ctrl, false);
  util::GetParam(nh_priv_, "abs_alt_ctrl", abs_yaw_ctrl, false);

  util::GetParam(nh_priv_, "model_delay_s", time_delay_s, 0.262137);
  util::GetParam(nh_priv_, "model_cx", Cx, -0.576335778073963);
  util::GetParam(nh_priv_, "model_cy", Cy, -0.584975281133000);
  util::GetParam(nh_priv_, "update_freq", update_freq, 30.0);

  // We initialize PIDs through its nodehandle constructor,
  // The following will set some default values for the parameters if the user
  // does not specify them. This plays nice with Dynamic Reconfigure
  nh_pid_vx.setParam("p", nh_pid_vx.param("p", 0.15));
  nh_pid_vx.setParam("i", nh_pid_vx.param("i", 0.4));
  nh_pid_vx.setParam("d", nh_pid_vx.param("d", 0.0));
  nh_pid_vx.setParam("i_clamp", nh_pid_vx.param("i_clamp", 0.02));

  nh_pid_vy.setParam("p", nh_pid_vy.param("p", 0.15));
  nh_pid_vy.setParam("i", nh_pid_vy.param("i", 0.4));
  nh_pid_vy.setParam("d", nh_pid_vy.param("d", 0.0));
  nh_pid_vy.setParam("i_clamp", nh_pid_vy.param("i_clamp", 0.02));

  nh_pid_yaw.setParam("p", nh_pid_yaw.param("p", 0.5));
  nh_pid_yaw.setParam("i", nh_pid_yaw.param("i", 0.0));
  nh_pid_yaw.setParam("d", nh_pid_yaw.param("d", 0.0));
  nh_pid_yaw.setParam("i_clamp", nh_pid_yaw.param("i_clamp", 0.02));

  nh_pid_alt.setParam("p", nh_pid_alt.param("p", 0.5));
  nh_pid_alt.setParam("i", nh_pid_alt.param("i", 0.0));
  nh_pid_alt.setParam("d", nh_pid_alt.param("d", 0.0));
  nh_pid_alt.setParam("i_clamp", nh_pid_alt.param("i_clamp", 0.02));

  ROS_ASSERT(pid_vx && pid_vy && pid_yaw && pid_alt);

  pid_vx->init(nh_pid_vx);
  pid_vy->init(nh_pid_vy);
  pid_yaw->init(nh_pid_yaw);
  pid_alt->init(nh_pid_alt);

  velx_model = boost::make_shared<bebop_vel_ctrl::BebopVelocityTiltModel>(Cx, GRAV_CST);
  vely_model = boost::make_shared<bebop_vel_ctrl::BebopVelocityTiltModel>(Cy, -GRAV_CST);

  bebop_sync.registerCallback(boost::bind(&BebopVelCtrl::BebopSyncCallback, this, _1, _2, _3));
}

void BebopVelCtrl::BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                                     const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr,
                                     const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr)
{
  bebop_recv_time = ros::Time::now();

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
    beb_yaw_ref_rad = -att_ptr->yaw;
    beb_param_recv = true;
  }

  beb_roll_rad = att_ptr->roll;
  beb_pitch_rad = -att_ptr->pitch;
  beb_yaw_rad = -att_ptr->yaw;
  beb_alt_m = alt_ptr->altitude;
  ROS_INFO_STREAM("Current Bebop State: RPY & ALT: "
                  << angles::to_degrees(beb_roll_rad) << " "
                  << angles::to_degrees(beb_pitch_rad) << " "
                  << angles::to_degrees(beb_yaw_rad) << " "
                  << beb_alt_m << " ");

  // Bebop Speed is in world coordinates (ESD coords (we convert it to ENU))
  // yaw is already converted (with respect to magnetic north)
  const double vx_enu = speed_ptr->speedX;
  const double vy_enu = -speed_ptr->speedY;
  const double vz_enu = -speed_ptr->speedZ;

  beb_vx_m = cos(beb_yaw_rad) * vx_enu + sin(beb_yaw_rad) * vy_enu;
  beb_vy_m = -sin(beb_yaw_rad) * vx_enu + cos(beb_yaw_rad) * vy_enu;

  ROS_INFO_STREAM("Current Bebop Velcoities: XYZ: "
                  << beb_vx_m << " "
                  << beb_vy_m << " "
                  << beb_vz_m << " ");
  beb_vyaw_rad = 0.0; // this is unknown


  velx_model->Simulate(time_delay_s, 0.05, beb_vx_m, beb_pitch_rad);
  vely_model->Simulate(time_delay_s, 0.05, beb_vy_m, beb_roll_rad);

  ROS_INFO_STREAM("Simulated Bebop Velocities: XY: "
                  << velx_model->GetVel() << " " << vely_model->GetVel());
  ROS_WARN_STREAM("Last predicted bebop vels: XY:"
                  << beb_vx_pred_m << " " << beb_vy_pred_m);

  beb_vx_pred_m = beb_vx_m;
  beb_vy_pred_m = beb_vy_m;
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
  if ((t_now - bebop_recv_time).toSec() > 1.0) return;

  // CLAMP Input Setpoints
  geometry_msgs::Twist setpoint = *twist_ptr_;
  CLAMP(setpoint.linear.x, -max_linear_vel, max_linear_vel);
  CLAMP(setpoint.linear.y, -max_linear_vel, max_linear_vel);
  CLAMP(setpoint.linear.z, -max_vertical_vel, max_vertical_vel);
  CLAMP(setpoint.angular.z, -max_angular_vel, max_angular_vel);

  // PID Control Loop
  const ros::Duration& dt = t_now - pid_last_time;
  const double pitch_ref = pid_vx->computeCommand(setpoint.linear.x - velx_model->GetVel(), dt);
  const double roll_ref  = pid_vy->computeCommand(setpoint.linear.y - vely_model->GetVel(), dt);

  // If abs yaw ctrl is set, setpoint.angular.z is an angle
  const double vyaw_ref = (abs_yaw_ctrl) ?
        pid_yaw->computeCommand(angles::normalize_angle(setpoint.angular.z - beb_yaw_rad), dt) :
        setpoint.angular.z;

  // If abs alt ctrl is set, setpoint.linear.z is an altitude
  const double vz_ref = (abs_alt_ctrl) ?
        pid_alt->computeCommand(setpoint.linear.z - beb_alt_m, dt) :
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
  FILTER_SMALL_VALS(ctrl_twist_.linear.x , 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.y , 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.z , 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.angular.z , 0.01);

  // Simulate Bebop's velocity given the reference tilt to update (predict) feedback
  // this to compensate for low frequency velocity feedback
  // We again convert from normalized values to angles to take into effect any clamp/filtering effects
  velx_model->Reset(velx_model->GetVel(), CLAMP(ctrl_twist_.linear.x * beb_maxtilt_rad, -beb_maxtilt_rad, beb_maxtilt_rad));
  velx_model->Simulate(dt.toSec());

  vely_model->Reset(vely_model->GetVel(), CLAMP(ctrl_twist_.linear.y * beb_maxtilt_rad, -beb_maxtilt_rad, beb_maxtilt_rad));
  vely_model->Simulate(dt.toSec());

  pid_last_time = ros::Time::now();

  pub_ctrl_cmd_vel_.publish(ctrl_twist_);

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

  ros::Rate loop_rate(update_freq);

  pid_last_time = ros::Time::now();
  while (ros::ok())
  {
    try
    {
      if ((ros::Time::now() - bebop_recv_time).toSec() > 1.0)
      {
        ROS_WARN("[VCTL] Bebop state feedback is older than 1 second! Resetting.");
        Reset();
      }

      if ((ros::Time::now() - pid_last_time).toSec() > (5.0 / update_freq))
      {
        ROS_WARN("[VCTL] Input ctrl_cmd_vel is old or slow! Resetting.");
        Reset();
      }

      ros::spinOnce();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[VCTL] Missed loop update rate of " << update_freq);
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("[VCTL] Runtime Exception: " << e.what());
      Reset();
    }
  }
}
}  // namespace bebop_vel_ctrl
