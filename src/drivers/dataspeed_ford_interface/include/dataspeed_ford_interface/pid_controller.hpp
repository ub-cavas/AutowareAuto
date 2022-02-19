#ifndef DATASPEED_FORD_INTERFACE__PID_CONTROLLER_HPP_
#define DATASPEED_FORD_INTERFACE__PID_CONTROLLER_HPP_

#include <algorithm>

#include <common/types.hpp>
#include <dataspeed_ford_interface/visibility_control.hpp>

using autoware::common::types::float64_t;

namespace autoware
{
namespace dataspeed_ford_interface
{

/**
 * @brief Adapted from
 *  https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/master/dbw_mkz_twist_controller/src/PidControl.h
 *
 */
class DATASPEED_FORD_INTERFACE_LOCAL PIDController
{
public:
  PIDController(float64_t kp, float64_t ki, float64_t kd, float64_t min_val, float64_t max_val)
  : m_last_err(0.0),
    m_integral(0.0),
    m_prev_integral(0.0),
    m_kp(kp),
    m_ki(ki),
    m_kd(kd),
    m_min(std::min(min_val, max_val)),
    m_max(std::max(min_val, max_val)){};

  PIDController()
  : PIDController(
      0.0,
      0.0,
      0.0,
      -std::numeric_limits<float64_t>::infinity(),
      std::numeric_limits<float64_t>::infinity())
  {
  }

  void setGains(float64_t kp, float64_t ki, float64_t kd)
  {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  void setKp(float64_t kp) {
    m_kp = kp;
  }

  void setKi(float64_t ki) {
    m_ki = ki;
  }

  void setKd(float64_t kd) {
    m_kd = kd;
  }

  void setRange(float64_t min_val, float64_t max_val)
  {
    m_min = std::min(min_val, max_val);
    m_max = std::max(min_val, max_val);
  }

  void resetIntegrator() { m_integral = 0.0; }

  void revertIntegrator() { m_integral = m_prev_integral; }

  float64_t step(float64_t error, float64_t dt)
  {
    m_prev_integral = m_integral;
    float64_t integral = m_integral + error * dt;
    float64_t derivative = (error - m_last_err) / dt;
    float64_t y = m_kp * error + m_ki * m_integral + m_kd * derivative;

    if (y > m_max) {
      y = m_max;
    } else if (y < m_min) {
      y = m_min;
    } else {
      m_integral = integral;
    }

    m_last_err = error;
    return y;
  }


private:
  float64_t m_last_err;
  float64_t m_integral, m_prev_integral;
  float64_t m_kp, m_ki, m_kd;
  float64_t m_min, m_max;
};
}  // namespace dataspeed_ford_interface
}  // namespace autoware


#endif /* DATASPEED_FORD_INTERFACE__PID_CONTROLLER_HPP_ */
