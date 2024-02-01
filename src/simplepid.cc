#include <simplepid.hpp>

namespace EMIRO
{
    // Simple PID implementation
    PID::PID(const double &Kp, const double &Ki, double Kd)
    {
    }

    float PID::get_control(const double &set_point, const double &current_point)
    {
        float error = set_point - current_point;
        integral = integral + error;
        float control = Kp * error + Ki * integral + Kd * (error - pre_error);
        pre_error = error;
        return control;
    }
    PID::~PID()
    {
    }

    // Three axis PID implementation
    ThreeAxisPID::ThreeAxisPID(const Position &target_point, const double &Kp, const double &Ki, double Kd)
        : set_point(target_point), pid_x(Kp, Ki, Kd), pid_y(Kp, Ki, Kd), pid_z(Kp, Ki, Kd)
    {
    }

    void ThreeAxisPID::set_speed_limit(LinearSpeed limit_m_s)
    {
        if (limit_m_s.linear_x < 0.1f &&
            limit_m_s.linear_y < 0.1f &&
            limit_m_s.linear_z < 0.1f)
        {
            std::cout << C_YELLOW << S_BOLD << "Warning :" << C_RESET << " Speed limit too small. Set Speed limit " << C_RED << "[REJECTED]" << C_RESET << '\n';
            return;
        }
        speed_limit = limit_m_s;
        is_limit = true;
    }
    void ThreeAxisPID::set_speed_limit(float limit_m_s)
    {
        if (limit_m_s < 0.1f)
        {
            std::cout << C_YELLOW << S_BOLD << "Warning :" << C_RESET << " Speed limit too small. Set Speed limit " << C_RED << "[REJECTED]" << C_RESET << '\n';
            return;
        }
        speed_limit = {limit_m_s, limit_m_s, limit_m_s};
        is_limit = true;
    }
    void ThreeAxisPID::get_control(const Position &current_point, LinearSpeed &out_speed)
    {
        out_speed = {pid_x.get_control(set_point.x, current_point.x),
                     pid_y.get_control(set_point.y, current_point.y),
                     pid_z.get_control(set_point.z, current_point.z)};
        if (!is_limit)
            return;

        if (out_speed.linear_x > speed_limit.linear_x)
            out_speed.linear_x = speed_limit.linear_x;
        if (out_speed.linear_y > speed_limit.linear_y)
            out_speed.linear_y = speed_limit.linear_y;
        if (out_speed.linear_z > speed_limit.linear_z)
            out_speed.linear_z = speed_limit.linear_z;
    }
    ThreeAxisPID::~ThreeAxisPID()
    {
    }
}