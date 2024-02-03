#pragma once

#include <copter.hpp>
#include <gps.hpp>
#include <mat.hpp>
#include <simplepid.hpp>

namespace EMIRO {
class Control {
  public:
    /**
     * @brief Manual Control using SimplePID
     *
     * @note Parameters must be set
     * @see Control(std::shared_ptr<Copter> copter, std::shared_ptr<Logger>
     * logger, std::shared_ptr<GPS> gps);
     */
    Control();

    /**
     * @brief Control Constructor
     *
     * @param copter    Copter object
     * @param logger    Logger object for event logging
     * @param gps       GPS object to convert Copter Local degree into Global
     * position
     *
     * @note
     *  - Copter object must be initialized
     */
    Control(std::shared_ptr<Copter> copter, std::shared_ptr<Logger> logger,
            std::shared_ptr<GPS> gps);

#pragma region Getters
    /**
     * @brief Get the linear speed limit
     *
     * @return float& speed limit (m/s)
     */
    float &get_linear_speed();

    /**
     * @brief Get the rotation speed limit
     *
     * @return float& rotation speed limit (deg/s)
     */
    int get_rotate_speed();
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
    void set_linear_speed_limit(const float &limit_m_s = 5.f);

    /**
     * @brief Set the limit of copter rotation speed in x, y and z axis
     *
     * @param limit_rad_s rotation limit (deg/s)
     *
     * @note
     *  - limit_rad_s = 1 - 180
     * @default limit_rad_s = 10
     */
    void set_rotate_speed_limit(const int &limit_deg_s = 10);
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
    void go(const float &x, const float &y, const float &z, const int &yaw,
            const float &pos_precision = 0.2f, const int &yaw_precision = 5);

    ~Control();

  private:
    std::shared_ptr<Copter> copter;
    std::shared_ptr<Logger> logger;
    std::shared_ptr<GPS> _gps;
    bool is_init = false;

    float speed_limit = 1.0f;              // m/s
    float rotate_speed_limit = 1.55;       // rad/s
    float vx = 0.0f, vy = 0.0f, vz = 0.0f; // m/s
    float avx = 0, avy = 0, avz = 0;       // rad/s

    inline void check_init() {
        if (!is_init) {
            copter->Land();
            throw std::runtime_error("Bad control class implementation.");
        }
    }

    void go(bool yaw_control = true);
};
} // namespace EMIRO