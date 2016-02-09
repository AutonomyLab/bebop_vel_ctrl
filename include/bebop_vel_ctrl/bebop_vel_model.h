#ifndef BEBOP_VEL_MODEL_H
#define BEBOP_VEL_MODEL_H

#include <cassert>
#include <cmath>

#ifndef CLAMP
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#endif

#ifndef FILTER_SMALL_VALS
#define FILTER_SMALL_VALS(x, eps) (x = ((fabs((x)) < (eps)) ? 0.0 : (x)))
#endif

#ifndef GRAV_CTS
#define GRAV_CST 9.81 // g
#endif

namespace bebop_vel_ctrl
{

/* Generic 1st order non-linear model
 *
 * d_v = -C * v + G * tan(tilt)
 *
 * For Bebop:
 *
 * Cx: -0.576335778073963
 * Cy: -0.584975281133000
 * Gx: 9.81
 * Gy: -9.81
 *
 * */
class BebopVelocityTiltModel
{
private:
  double c_;
  double g_;
  double vel_m_;
  double tilt_rad_;

public:
  BebopVelocityTiltModel(const double& c, const double& g);

  BebopVelocityTiltModel(const double &c, const double &g,
                         const double &vel_init_m, const double& tilt_init_rad);

  inline double GetVel() const {return vel_m_;}
  inline double GetTilt() const {return tilt_rad_;}

  void Reset(const double &vel_init_m, const double& tilt_init_rad);
  void Reset();

  /* Simulate the dynamic system for one time step dt
   * from current tilt angle and current velocity
   * */
  inline void Simulate(const double& dt_s)
  {
    assert( (dt_s > 0.0) && (fabs(dt_s) > 1e-6) );
    const double dv = (-c_ * vel_m_) + (g_ * tan(tilt_rad_));
    vel_m_ += (dv * dt_s);
  }

  /* Simulate the dynamic system assuming a fixed input (tilt_rad_)
   * for t seconds with dt steps
   *
   * Instead of discrizing the cont-time model, we perform
   * peice-wise integeration of the first order system
   *
   * */
  void Simulate(const double& duration_s, const double& dt_s);

  /* Overloaded function, also sets the initial conditions */
  void Simulate(const double &duration_s, const double &dt_s,
                const double &vel_init_m, const double& tilt_init_rad);
};

}  // namespace bebop_vel_ctrl

#endif
