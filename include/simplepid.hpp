#pragma once

#include <types.hpp>

namespace EMIRO
{
  class PID
  {

  public:
    PID(const double &Kp, const double &Ki, double Kd);

    /**
     * @brief Get the PID output
     *
     * @param set_point Desired point
     * @param current_point Current point
     *
     * @return float
     */
    float get_control(const double &set_point, const double &current_point);

    ~PID();

  private:
    double Kp, Ki, Kd;
    double pre_error;
    double integral;
  };

  class ThreeAxisPID
  {

  public:
    ThreeAxisPID(const Position &target_point, const double &Kp,
                 const double &Ki, double Kd);

    /**
     * @brief Set the copter maks speed (m/s)
     *
     * @param limit_m_s copter maks speed in linear x, y, z
     */
    void set_speed_limit(LinearSpeed limit_m_s);

    /**
     * @brief Set the copter maks speed (m/s)
     *
     * @param limit_m_s copter maks speed in linear x=y=z
     */
    void set_speed_limit(float limit_m_s);

    /**
     * @brief Get PID control for three axis (x, y, z)
     *
     * @param current_point Current copter position
     * @param out_speed Speed output in linear x, y, z
     */
    void get_control(Position &current_point, LinearSpeed &out_speed);

    ~ThreeAxisPID();

  private:
    PID pid_x, pid_y, pid_z;
    Position set_point;
    LinearSpeed speed_limit{0.0f, 0.0f, 0.0f};
    bool is_limit = false;
  };
} // namespace EMIRO