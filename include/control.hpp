#pragma once

#include <copter.hpp>
#include <gps.hpp>
#include <mat.hpp>
#include <simplepid.hpp>

namespace EMIRO
{
    class Control
    {
    public:
        static Control &get()
        {
            static Control control;
            return control;
        }

#pragma region Getters
        /**
         * @brief Get the linear speed limit
         *
         * @return float& speed limit (m/s)
         */
        static float get_linear_speed();

        /**
         * @brief Get the rotation speed limit
         *
         * @return float& rotation speed limit (deg/s)
         */
        static int get_rotate_speed();
#pragma endregion

#pragma region Setters
        /**
         * @brief Set the limit of copter speed in linear axis
         *
         * @param limit_m_s speed limit (m/s)
         *
         * @note minimum speed is 0.1m/s
         *
         * @default limit_m_s = 5
         */
        static void set_linear_speed_limit(const float &limit_m_s = 5.f);

        /**
         * @brief Set the limit of copter rotation speed in x, y and z axis
         *
         * @param limit_rad_s rotation limit (deg/s)
         *
         * @note
         *  - limit_rad_s = 1 - 180
         * @default limit_rad_s = 10
         */
        static void set_rotate_speed_limit(const int &limit_deg_s = 10);

        /**
         * @brief Set PID control parameters
         *
         * @param Kp
         * @param Ki
         * @param Kd
         *
         * @default
         *  - Kp = 0.5
         *  - Ki = 0.0
         *  - Kd = 0.05
         */
        static void set_PID(const double &Kp, const double &Ki, const double &Kd);
#pragma endregion

        /**
         * @brief Go to target point (x, y, z, yaw)
         *
         * @param x     position in axis x respect to copter home position (m)
         * @param y     position in axis y respect to copter home position (m)
         * @param z     position in axis z respect to copter home position (m)
         * @param yaw   copter yaw angle (degree)
         * @param pos_precision position precision (m)
         * @param yaw_precision yaw precision (degree)
         *
         * @default
         *  - pos_precision = 0.2
         *  - yaw_precision = 5
         */
        static void go(const float &x, const float &y, const float &z, const int &yaw, const float &pos_precision = .2f, const int &yaw_precision = 5);

    private:
        Control();
        ~Control();
        float control_get_linear_speed() const;
        int control_get_rotate_speed() const;
        void control_set_linear_speed_limit(const float &limit_m_s) const;
        void control_set_rotate_speed_limit(const int &limit_deg_s) const;
        void control_set_PID(const double &Kp,
                             const double &Ki,
                             const double &Kd);
        void control_go(const float &x,
                        const float &y,
                        const float &z,
                        const int &yaw,
                        const float &pos_precision,
                        const int &yaw_precision);

        float Kp = .5f, Ki = 0.f, Kd = .05f;
        mutable float linear_speed_limit = 1.0f;  // m/s
        mutable float rotate_speed_limit = 1.55f; // rad/s
        float vx = 0.f, vy = 0.f, vz = 0.f;       // m/s
        float avx = 0.f, avy = 0.f, avz = 0.f;    // rad/s

        void control_go(bool yaw_control = true);
    };

    class BaseControl
    {
    public:
        virtual void get() = 0;
        virtual bool go() = 0;
        virtual ~BaseControl() = default;
    };

    class PIDControl : public BaseControl
    {
    public:
        PIDControl(const Position &target_point, const double &Kp, const double &Ki, const double &Kd);
        ~PIDControl();
        void get() override;
        bool go() override;
    };
} // namespace EMIRO