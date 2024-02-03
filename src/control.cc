#include <control.hpp>

namespace EMIRO {
Control::Control() {}

Control::Control(std::shared_ptr<Copter> copter, std::shared_ptr<Logger> logger,
                 std::shared_ptr<GPS> gps)
    : _gps(gps), copter(copter), logger(logger) {
    _gps->lock_pos();
    is_init = true;
}

float &Control::get_linear_speed() { return linear_speed_limit; }

int Control::get_rotate_speed() { return rotate_speed_limit * 180 / M_PI; }

void Control::set_linear_speed_limit(const float &limit_m_s) {
    if (limit_m_s < 0.1f) {
        std::cout << C_YELLOW << S_BOLD << "Warning :" << C_RESET
                  << " Speed limit too small. Set Speed limit " << C_RED
                  << "[REJECTED]" << C_RESET << '\n';
        return;
    }
    linear_speed_limit = limit_m_s;
}

void Control::set_rotate_speed_limit(const int &limit_deg_s) {
    if (limit_deg_s < 1) {
        std::cout << C_YELLOW << S_BOLD << "Warning :" << C_RESET
                  << "Rotation limit too small. Set Speed limit" << C_RED
                  << "[REJECTED]" << C_RESET << '\n';
        return;
    } else if (limit_deg_s > 180) {
        std::cout << C_YELLOW << S_BOLD << "Warning :" << C_RESET
                  << "Rotation limit too large (>180). Set Speed limit" << C_RED
                  << "[REJECTED]" << C_RESET << '\n';
        return;
    }
    rotate_speed_limit = limit_deg_s * M_PI / 180.0f;
}

void Control::set_PID(const double &Kp, const double &Ki, const double &Kd) {
    if (Kp < 0.f || Ki < 0.f || Kd < 0.f) {
        logger->write_show(
            LogLevel::ERROR,
            "Invalid PID (%.2f, %.2f, %.2f). PID Kp, Ki, Kd must be positive",
            Kp, Ki, Kd);
        return;
    }
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void Control::go(bool yaw_control) {
    check_init();
    LinearSpeed speed = {vx, vy, vz};
    _gps->convert(speed);
    if (yaw_control)
        copter->set_vel(vx, vy, vz, avx, avy, avz);
    else
        copter->set_vel(vx, vy, vz, 0.0f, 0.0f, 0.0f);
}

void Control::go(const float &x, const float &y, const float &z, const int &yaw,
                 const float &pos_precision, const int &angular_precision) {
    check_init();
    logger->write_show(LogLevel::INFO, "Go : %.2f, %.2f, %.2f, %d", x, y, z,
                       yaw);

    // Loop variable
    ros::Rate r(5);
    Position pos;
    Quaternion quat;
    Euler eul;
    float _yaw = 0.f;

    Position target_point = {x, y, z};
    ThreeAxisPID pid(target_point, Kp, Ki, Kd);
    pid.set_speed_limit(linear_speed_limit);

    std::cout << std::fixed << std::setprecision(3);
    while (ros::ok()) {
        // Get current position and orientation
        copter->get_pose(&pos, &quat);
        eul = to_euler(quat.w, quat.x, quat.y, quat.z);
        eul.roll *= (180.0f / M_PI);
        eul.pitch *= (180.0f / M_PI);
        eul.yaw *= (180.0f / M_PI);
        eul.yaw = eul.yaw > 180 ? eul.yaw - 360 : eul.yaw;
        _yaw = eul.yaw;

        // Close if position in target zone
        if (std::fabs(pos.x - x) < pos_precision &&
            std::fabs(pos.y - y) < pos_precision &&
            std::fabs(pos.z - z) < pos_precision &&
            std::fabs(eul.yaw - yaw) < angular_precision)
            break;

        LinearSpeed _out_pid;
        pid.get_control(pos, _out_pid);

        float diff_x = x - pos.x;
        float diff_y = y - pos.y;
        float diff_z = z - pos.z;
        float diff_yaw = yaw - eul.yaw;

        diff_yaw = diff_yaw >= 180 ? -(360 - diff_yaw) : diff_yaw;

        avz = diff_yaw * 3.14 / 180.f;
        if (std::fabs(avz) > rotate_speed_limit)
            avz = (avz > 0.f) ? rotate_speed_limit : -rotate_speed_limit;
        else if (std::fabs(eul.yaw - yaw) < angular_precision)
            avz = 0.f;

        // go(true);
        copter->set_vel(_out_pid.linear_x, _out_pid.linear_y, _out_pid.linear_z,
                        0.f, 0.f, avz);

        // Print position
        std::cout << C_MAGENTA << S_BOLD << " >>> " << C_RESET << "Target ("
                  << diff_x << ", " << diff_y << ", " << diff_z << ", "
                  << diff_yaw << "°)     \r" << std::flush;

        ros::spinOnce();
        r.sleep();
    }
    logger->write_show(
        LogLevel::INFO,
        "Reached (%.2f, %.2f, %.2f, %d°) => (%.2f, %.2f, %.2f, %d°)", x, y, z,
        pos.x, pos.y, pos.z, (int)yaw, (int)_yaw);
}

Control::~Control() {}
} // namespace EMIRO