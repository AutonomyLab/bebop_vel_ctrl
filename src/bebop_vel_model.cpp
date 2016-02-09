#include <stdexcept>

#include "bebop_vel_ctrl/bebop_vel_model.h"

namespace bebop_vel_ctrl {

BebopVelocityTiltModel::BebopVelocityTiltModel(const double& c, const double& g)
  : c_(c), g_(g), vel_m_(0.0), tilt_rad_(0.0)
{}

BebopVelocityTiltModel::BebopVelocityTiltModel(const double &c, const double &g,
                                               const double &vel_init_m, const double& tilt_init_rad)
  : c_(c), g_(g), vel_m_(vel_init_m), tilt_rad_(tilt_init_rad)
{}


void BebopVelocityTiltModel::Reset(const double &vel_init_m, const double& tilt_init_rad)
{
  vel_m_ = vel_init_m;
  tilt_rad_ = tilt_init_rad;
}

void BebopVelocityTiltModel::Reset()
{
  Reset(0.0, 0.0);
}

void BebopVelocityTiltModel::Simulate(const double& duration_s, const double& dt_s)
{
  if (duration_s < 0.0)
  {
    throw std::runtime_error("Negative duration");
  }
  double t = 0.0;
  while (t < duration_s)
  {
    Simulate(dt_s);
    t += dt_s;
  }
}

void BebopVelocityTiltModel::Simulate(const double &duration_s, const double &dt_s,
                                      const double &vel_init_m, const double& tilt_init_rad)
{
  Reset(vel_init_m, tilt_init_rad);
  Simulate(duration_s, dt_s);
}


}  // namespace bebop_vel_ctrl
