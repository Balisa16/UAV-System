#include <copter.hpp>

namespace EMIRO
{
    const std::string COPTER_DIR = std::getenv("EMIRO_PATH");

    Copter::Copter()
    {
        is_init_pubs_subs = false;
        is_init_frame = false;
        // traj_logger.init("Track", FileType::CSV, "x pos(m),y pos(m),z
        // pos(m),w orient(rad),x orient(rad),y orient(rad),z orient(rad)");
        traj_logger.init("Track", FileType::CSV);
        traj_logger.start(true);
    }

    bool
    Copter::init(int argc, char **argv, std::string copter_name, std::string log_name, FileType log_type)
    {
        static bool _init_ros = false;
        if (!_init_ros)
        {
            ros::init(argc, argv, copter_name);
            _init_ros = true;
        }
        return get().copter_init(log_name, log_type);
    }

    // Copter
    void
    Copter::init_frame(float timeout_s)
    {
        get().copter_init_frame(timeout_s);
    }

    bool
    Copter::FCUconnect(float timeout_s)
    {
        return get().copter_FCUconnect(timeout_s);
    }

    bool
    Copter::FCUstart(float timeout_s)
    {
        return get().copter_FCUstart(timeout_s);
    }

    bool Copter::PreArmedCheck(float timeout_s)
    {
        uint64_t _cnt = 5 * timeout_s;
        return get().copter_PreArmedCheck(_cnt);
    }

    void Copter::waitHDOP(float hdop_limit, u_int64_t duration_ms)
    {
        if (hdop_limit > 2.0f)
        {
            get_logger().write_show(LogLevel::INFO, "HDOP limit too high. Setting to 2.0");
            hdop_limit = 2.0f;
        }
        get().copter_waitHDOP(hdop_limit, duration_ms);
    }

    int
    Copter::set_mode(FlightMode mode)
    {
        return get().copter_set_mode(mode);
    }

    void
    Copter::set_vel(const float &vx, const float &vy, const float &vz, const float &avx, const float &avy, const float &avz)
    {
        get().copter_set_vel(vx, vy, vz, avx, avy, avz);
    }

    void
    Copter::set_vel(geometry_msgs::Twist &cmd_vel)
    {
        get().copter_set_vel(cmd_vel.linear.x,
                             cmd_vel.linear.y,
                             cmd_vel.linear.z,
                             cmd_vel.angular.x,
                             cmd_vel.angular.y,
                             cmd_vel.angular.z);
    }

    bool
    Copter::Arming()
    {
        return get().copter_Arming();
    }

    int
    Copter::takeoff(float takeoff_alt, float tolerance)
    {
        return get().copter_takeoff(takeoff_alt, tolerance);
    }

    // int
    // Copter::just_takeoff(float takeoff_alt, float _yaw)
    // {
    //     return get().copter_just_takeoff(takeoff_alt, _yaw);
    // }

    // int
    // Copter::takeoff2(WayPoint wp)
    // {
    //     return get().copter_takeoff2(wp);
    // }

    void
    Copter::Go(WayPoint &wp, bool show, std::string header)
    {
        return get().copter_Go(wp, show, header);
    }

    float
    Copter::get_alt()
    {
        return get().copter_get_alt();
    }

    void
    Copter::Go_Land(WayPoint wp, float tolerance)
    {
        get().copter_Go_Land(wp, tolerance);
    }

    void
    Copter::Land(float ground_tolerance)
    {
        get().copter_Land(ground_tolerance);
    }

    int
    Copter::set_speed(float speed_mps)
    {
        return get().copter_set_speed(speed_mps);
    }

    void
    Copter::set_ekf_source(EKFSource source)
    {
        get().copter_set_ekf_source(source);
    }

    void Copter::realign_viso()
    {
        get().copter_realign_viso();
    }

    void
    Copter::set_ekf_origin(float lat, float lnt, float alt)
    {
        get().copter_set_ekf_origin(lat, lnt, alt);
    }

    int
    Copter::set_home(float lat, float lnt, float alt)
    {
        return get().copter_set_home(lat, lnt, alt);
    }

    bool
    Copter::is_reached(WayPoint dest, float tolerance)
    {
        return get().copter_is_reached(dest, tolerance);
    }

    bool
    Copter::check_alt(float dist_alt, float tolerance)
    {
        return get().copter_check_alt(dist_alt, tolerance);
    }

    WayPoint
    Copter::calc_transition(WayPoint start_point, WayPoint stop_point,
                            float copter_deg, float copter_alt)
    {
        return get().copter_calc_transition(start_point, stop_point, copter_deg,
                                            copter_alt);
    }

    void
    Copter::set_rc(int channel, int pwm)
    {
        get().copter_set_rc(channel, pwm);
    }

    void
    Copter::get_pose(Position *pos, Quaternion *quat)
    {
        get().copter_get_pose(pos, quat);
    }

    float
    Copter::get_yaw()
    {
        return get().copter_get_yaw();
    }

    void
    Copter::get_position(WayPoint &pose_ref)
    {
        get().copter_get_position(pose_ref);
    }

    float
    Copter::get_hdop()
    {
        return get().copter_get_hdop();
    }

    float
    Copter::get_vdop()
    {
        return get().copter_get_vdop();
    }

    int
    Copter::get_satellites_num()
    {
        return get().copter_get_satellites_num();
    }

    GPS_FIX_TYPE
    Copter::get_gps_fix_type()
    {
        return get().copter_get_gps_fix_type();
    }

    WayPoint Copter::get_takeoff_position()
    {
        return get().takeoff_wp;
    }

    Environment
    Copter::get_current_mission()
    {
        return get().copter_get_current_mission();
    }

    void
    Copter::go_rtl(float alt, float tolerance)
    {
        get().copter_Go_RTL(alt, tolerance);
    }

    void Copter::set_yaw(YawMode _yaw_mode)
    {
        get().yaw_mode = _yaw_mode;
    }

#pragma region Initialize

    bool
    Copter::copter_init(const std::string logger_name, const FileType logger_type)
    {
        get_logger().init(logger_name, logger_type);
        get_logger().start(true);

        if (!this->is_init_pubs_subs)
        {
            // ROS Service Client
            command_client = get_nh()->serviceClient<mavros_msgs::CommandLong>(
                "/mavros/cmd/command");
            set_mode_client = get_nh()->serviceClient<mavros_msgs::SetMode>(
                "/mavros/set_mode");
            arming_client = get_nh()->serviceClient<mavros_msgs::CommandBool>(
                "/mavros/cmd/arming");
            takeoff_client = get_nh()->serviceClient<mavros_msgs::CommandTOL>(
                "/mavros/cmd/takeoff");
            land_client = get_nh()->serviceClient<mavros_msgs::CommandTOL>(
                "/mavros/cmd/land");

            // RC (Remote Control) Publisher
            cmd_rc_pub = get_nh()->advertise<mavros_msgs::RCIn>("/mavros/rc/in", 2);

            cmd_rc_override_pub = get_nh()->advertise<mavros_msgs::OverrideRCIn>(
                "mavros/rc/override", 10);

            // ROS Subscriber/Publisher
            cmd_pos_pub = get_nh()->advertise<geometry_msgs::PoseStamped>(
                "/mavros/setpoint_position/local", 20);
            gps_pos_pub = get_nh()->advertise<geographic_msgs::GeoPoseStamped>(
                "/mavros/setpoint_position/global", 10);
            state_sub = get_nh()->subscribe<mavros_msgs::State>(
                "/mavros/state", 10,
                [this](const mavros_msgs::State::ConstPtr &msg)
                {
                    current_state_g = *msg;
                });
            cmd_pos_sub_local = get_nh()->subscribe<geometry_msgs::PoseStamped>(
                "/mavros/local_position/pose", 5,
                [this](const geometry_msgs::PoseStamped::ConstPtr &msg)
                {
                    pose_data_local = *msg;
                    traj_logger.write(
                        LogLevel::INFO, "%f,%f,%f,%f,%f,%f,%f", pose_data_local.pose.position.x,
                        pose_data_local.pose.position.y, pose_data_local.pose.position.z,
                        pose_data_local.pose.orientation.w, pose_data_local.pose.orientation.x,
                        pose_data_local.pose.orientation.y, pose_data_local.pose.orientation.z);
                    timestamp = pose_data_local.header.stamp;
                });
            gps_raw_sub = get_nh()->subscribe<mavros_msgs::GPSRAW>(
                "/mavros/gpsstatus/gps1/raw", 5,
                [this](const mavros_msgs::GPSRAW::ConstPtr &msg)
                {
                    gps_raw = *msg;
                });

            cmd_vel_pub = get_nh()->advertise<geometry_msgs::Twist>(
                "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
            get_logger().write_show(LogLevel::INFO,
                                    "Publisher and Subscriber Initialized");
        }
        else
            get_logger().write_show(LogLevel::WARNING,
                                    "Publisher and Subscriber Already Initialized");

        get_logger().write_show(LogLevel::INFO, "Initializing GPS");

        GPS::init();
        return true;
    }

    void
    Copter::copter_init_frame(const float timeout_s)
    {
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;

        if (!is_init_frame)
        {
            ros::Rate _init_rate(5);
            get_logger().wait("Initialized Copter");
            while (ros::ok() && _timeout_int)
            {
                ros::spinOnce();
                _timeout_rate.sleep();
                _timeout_int--;
            }
            get_logger().wait_success();

            copter_get_position(local_frame);
            if (local_frame.x == 0.000f && local_frame.y == 0.000f && local_frame.z == 0.000f)
                get_logger().write_show(LogLevel::WARNING,
                                        "Local Frame looks unreasonable");
        }
    }
#pragma endregion

#pragma region FCU
    bool
    Copter::copter_FCUconnect(const float timeout_s) const
    {
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;

        while (ros::ok() && !current_state_g.connected && _timeout_int)
        {
            _timeout_int--;
            ros::spinOnce();
            _timeout_rate.sleep();
        }
        if (current_state_g.connected)
            return true;
        return false;
    }

    bool
    Copter::copter_FCUstart(const float timeout_s) const
    {
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;

        get_logger().wait("Waiting for GUIDED");
        while (ros::ok() && current_state_g.mode != "GUIDED" && _timeout_int)
        {
            _timeout_int--;
            ros::spinOnce();
            _timeout_rate.sleep();
        }
        get_logger().wait_success();

        if (current_state_g.mode == "GUIDED")
            return true;
        return false;
    }
#pragma endregion

    bool Copter::copter_PreArmedCheck(uint64_t &cnt) const
    {
        // Validation position
        WayPoint _wp;
        ros::Rate _rate(5);
        copter_get_position(_wp);
        get_logger().wait("Waiting for Valid Position");
        while (std::fabs(_wp.x) < 0.0001 && std::fabs(_wp.y) < 0.0001 && std::fabs(_wp.z) < 0.0001 && ros::ok() && cnt)
        {
            copter_get_position(_wp);
            cnt--;
            ros::spinOnce();
            _rate.sleep();
        }
        if (std::fabs(_wp.x) < 0.0001 && std::fabs(_wp.y) < 0.0001 && std::fabs(_wp.z) < 0.0001)
        {
            get_logger().wait_failed();
            get_logger().write_show(LogLevel::ERROR, "Position Validation Failed");
            return false;
        }
        else
        {
            get_logger().wait_success();
            get_logger().write_show(LogLevel::INFO, "Position Validation Success");
        }

        // Validation Mode
        get_logger().wait("Waiting for GUIDED Mode");
        while (current_state_g.mode != "GUIDED" && ros::ok() && cnt)
        {
            cnt--;
            ros::spinOnce();
            _rate.sleep();
        }
        if (current_state_g.mode == "GUIDED")
        {
            get_logger().wait_success();
            get_logger().write_show(LogLevel::INFO, "Mode Validation Success");
        }
        else
        {
            get_logger().wait_failed();
            get_logger().write_show(LogLevel::ERROR, "Mode Validation Failed");
            return false;
        }

        return true;
    }

    void
    Copter::copter_waitHDOP(float hdop_limit, u_int64_t duration_ms) const
    {
        ros::Rate _rate(5);
        get_logger().wait("Waiting for HDOP");
        while (ros::ok() && gps_raw.eph / 1E2F > hdop_limit && duration_ms)
        {
            ros::spinOnce();
            _rate.sleep();
            duration_ms -= 200;
        }
        get_logger().wait_success();
    }

#pragma region Pose
    Quaternion
    Copter::_to_quaternion(const float roll_rate, const float pitch_rate, const float yaw_rate) const
    {
        const float yaw = yaw_rate * (M_PI / 180);
        const float pitch = pitch_rate * (M_PI / 180);
        const float roll = roll_rate * (M_PI / 180);

        const float cy = cos(yaw * 0.5);
        const float sy = sin(yaw * 0.5);
        const float cr = cos(roll * 0.5);
        const float sr = sin(roll * 0.5);
        const float cp = cos(pitch * 0.5);
        const float sp = sin(pitch * 0.5);

        const float qw = cy * cr * cp + sy * sr * sp;
        const float qx = cy * sr * cp - sy * cr * sp;
        const float qy = cy * cr * sp + sy * sr * cp;
        const float qz = sy * cr * cp - cy * sr * sp;

        return {qw, qx, qy, qz};
    }

    geometry_msgs::Point
    Copter::_enu_2_local(const nav_msgs::Odometry current_pose_enu) const
    {
        const float x = current_pose_enu.pose.pose.position.x;
        const float y = current_pose_enu.pose.pose.position.y;
        const float z = current_pose_enu.pose.pose.position.z;
        const float deg2rad = (M_PI / 180);
        geometry_msgs::Point current_pos_local;
        current_pos_local.x = x * cos((local_offset_g - 90) * deg2rad) - y * sin((local_offset_g - 90) * deg2rad);
        current_pos_local.y = x * sin((local_offset_g - 90) * deg2rad) + y * cos((local_offset_g - 90) * deg2rad);
        current_pos_local.z = z;
        return current_pos_local;
    }
#pragma endregion

#pragma region Getter
    geometry_msgs::Point
    Copter::_get_hexa_point() const
    {
        return pose_data_local.pose.position;
    }

    Environment
    Copter::copter_get_current_mission() const
    {
        return misi_mode;
    }

    void
    Copter::copter_get_pose(Position *pos, Quaternion *quat) const
    {
        pos->x = pose_data_local.pose.position.x;
        pos->y = pose_data_local.pose.position.y;
        pos->z = pose_data_local.pose.position.z;

        quat->w = pose_data_local.pose.orientation.w;
        quat->x = pose_data_local.pose.orientation.x;
        quat->y = pose_data_local.pose.orientation.y;
        quat->z = pose_data_local.pose.orientation.z;
    }

    float
    Copter::copter_get_alt() const
    {
        return pose_data_local.pose.position.z;
    }

    void
    Copter::copter_get_position(WayPoint &pose_ref) const
    {
        pose_ref = {(float)pose_data_local.pose.position.x,
                    (float)pose_data_local.pose.position.y,
                    (float)pose_data_local.pose.position.z, get_yaw()};
    }

    float Copter::copter_get_hdop() const
    {
        return gps_raw.eph;
    }

    float Copter::copter_get_vdop() const
    {
        return gps_raw.epv;
    }

    int
    Copter::copter_get_satellites_num() const
    {
        return gps_raw.satellites_visible;
    }

    GPS_FIX_TYPE
    Copter::copter_get_gps_fix_type() const
    {
        return static_cast<GPS_FIX_TYPE>(gps_raw.fix_type);
    }

    float
    Copter::copter_get_yaw() const
    {
        Eigen::Quaternionf c_quat(pose_data_local.pose.orientation.w, pose_data_local.pose.orientation.x, pose_data_local.pose.orientation.y, pose_data_local.pose.orientation.z);
        float _yaw = std::atan2(2.0f * (c_quat.w() * c_quat.z() + c_quat.x() * c_quat.y()), 1.0f - 2.0f * (c_quat.y() * c_quat.y() + c_quat.z() * c_quat.z())) * 180.0f / M_PI;

        if (yaw_mode == YawMode::RELATIVE)
        {
            Eigen::Quaternionf r_quat = start_quat.inverse() * c_quat; // Relative
            _yaw = std::atan2(2.0f * (r_quat.w() * r_quat.z() + r_quat.x() * r_quat.y()), 1.0f - 2.0f * (r_quat.y() * r_quat.y() + r_quat.z() * r_quat.z())) * 180.0f / M_PI;
        }

        while (_yaw > 180.0)
        {
            _yaw -= 360.0;
        }
        while (_yaw < -180.0)
        {
            _yaw += 360.0;
        }
        return _yaw;
    }
#pragma endregion

#pragma region Go
    void
    Copter::_go_to(const geometry_msgs::Pose pose)
    {
        pose_stamped.header.stamp = timestamp;
        pose_stamped.pose = pose;

        cmd_pos_pub.publish(pose_stamped);
    }

    void
    Copter::_goto_xyz_rpy(const float x, const float y, const float z,
                          const float roll, const float pitch, const float yaw)
    {
        pose_data2.position.x = x;
        pose_data2.position.y = y;
        pose_data2.position.z = z;

        Quaternion _q = _to_quaternion(roll, pitch, yaw + M_PI_2);

        pose_data2.orientation.x = _q.x;
        pose_data2.orientation.y = _q.y;
        pose_data2.orientation.z = _q.z;
        pose_data2.orientation.w = _q.w;

        _go_to(pose_data2);
    }

    void
    Copter::copter_Go(WayPoint &wp, bool show, std::string header)
    {
        if (show)
            print_wp(header, wp);
        _goto_xyz_rpy(wp.x, wp.y, wp.z, 0, 0, wp.yaw);
    }
#pragma endregion

#pragma region Set_velocity
    void
    Copter::copter_set_vel(const float &vx, const float &vy, const float &vz,
                           const float &avx, const float &avy, const float &avz)
    {
        cmd_vel.linear.x = vx;
        cmd_vel.linear.y = vy;
        cmd_vel.linear.z = vz;

        cmd_vel.angular.x = avx;
        cmd_vel.angular.y = avy;
        cmd_vel.angular.z = avz;

        cmd_vel_pub.publish(cmd_vel);
    }

    void
    Copter::copter_set_vel(geometry_msgs::Twist &cmd_vel)
    {
        cmd_vel_pub.publish(cmd_vel);
    }
#pragma endregion

    bool
    Copter::copter_Arming()
    {
        _goto_xyz_rpy(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 10; i++)
        {
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        start_quat = {(float)pose_data_local.pose.orientation.w,
                      (float)pose_data_local.pose.orientation.x,
                      (float)pose_data_local.pose.orientation.y,
                      (float)pose_data_local.pose.orientation.z};

        get_logger().wait("Arming drone");
        mavros_msgs::CommandBool arm_request;
        arm_request.request.value = true;

        ros::Rate _internal_rate(2);
        int _timeout = 5 * 2; // 5 seconds timeout
        while (!current_state_g.armed && !arm_request.response.success && ros::ok())
        {
            arming_client.call(arm_request);
            cmd_pos_pub.publish(pose_stamped);
            _timeout--;
            if (_timeout <= 0)
                break;
            ros::spinOnce();
            _internal_rate.sleep();
        }

        if (arm_request.response.success)
        {
            status = Status::Armed;
            get_logger().wait_success();
            get_logger().write_show(LogLevel::INFO, "Drone armed successfully.");
            return true;
        }
        get_logger().wait_failed();
        get_logger().write_show(LogLevel::ERROR,
                                "Failed to arming drone. Code : %d",
                                arm_request.response.success);
        return false;
    }

    int
    Copter::copter_takeoff(float takeoff_alt, float tolerance)
    {
        ros::Rate _internal_rate(5);
        WayPoint _wp;

        // Get latest position
        while (ros::ok())
        {
            copter_get_position(_wp);
            if (std::fabs(_wp.x) > 0.00001 || std::fabs(_wp.y) > 0.00001 || std::fabs(_wp.z - takeoff_alt) > 0.00001)
                break;
            ros::spinOnce();
            _internal_rate.sleep();
        }

        // Arming
        if (!copter_Arming())
        {
            this->~Copter();
            exit(0);
        }

        _goto_xyz_rpy(_wp.x, _wp.y, takeoff_alt, 0, 0, _wp.yaw);
        for (int i = 0; i < 10; i++)
        {
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            _internal_rate.sleep();
        }

        // Call Takeoff
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = takeoff_alt;
        if (!takeoff_client.call(srv_takeoff))
        {
            get_logger().write_show(LogLevel::ERROR, "Failed Call Takeoff");
            this->~Copter();
            exit(EXIT_FAILURE);
        }

        // Takeoff
        uint32_t counter = 20 * 5; // 10 sec timeout
        while (ros::ok() && current_state_g.mode != "AUTO.TAKEOFF")
        {
            float _curr_alt = get().get_alt();
            counter--;
            std::cout << std::fixed << std::setprecision(3);
            std::cout << CLEAR_LINE << C_MAGENTA << S_BOLD << " >>> " << C_RESET << "Alt : " << _curr_alt << "m\t" << takeoff_alt - tolerance << " <= Alt <=  " << takeoff_alt + tolerance << std::flush;
            if (!counter)
            {
                // get_logger().wait_failed();
                std::cout << '\n';
                get_logger().write_show(LogLevel::ERROR, "Failed Takeoff [TIMEOUT]");
                this->~Copter();
                exit(0);
            }

            if (_curr_alt >= takeoff_alt - tolerance && _curr_alt <= takeoff_alt + tolerance)
            {
                std::cout << CLEAR_LINE;
                get_logger().write_show(LogLevel::INFO, "Success Takeoff");
                status = Status::Takeoff;
                break;
            }
            ros::spinOnce();
            _internal_rate.sleep();
        }
        // get_logger().wait_success();

        // Save current position as takeoff point
        copter_get_position(takeoff_wp);

        return 0;
    }

#pragma region Land
    bool
    Copter::_land(const float tolerance)
    {
        mavros_msgs::CommandTOL srv_land;
        if (land_client.call(srv_land) && srv_land.response.success)
        {
            WayPoint _wp;
            get_logger().wait("Waiting to Land");
            while (true)
            {
                if (!ros::ok())
                {
                    get_logger().wait_failed();
                    return false;
                }
                copter_get_position(_wp);
                if (std::fabs(_wp.z) < tolerance)
                    break;
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
            get_logger().wait_success();
            get_logger().write_show(LogLevel::INFO, "Success Land");
            status = Status::Land;
            return true;
        }
        get_logger().write_show(LogLevel::ERROR, "Land Failed : %d",
                                srv_land.response.success);
        return false;
    }

    void
    Copter::copter_Go_Land(WayPoint wp, float tolerance)
    {
        get_logger().wait("Go to Land Position");
        ros::Rate wait_land(2);
        while (ros::ok() && !copter_is_reached(wp, 0.2f))
        {
            get().copter_Go(wp);
            ros::spinOnce();
            wait_land.sleep();
        }

        ros::Duration(1).sleep();
        get_logger().write_show(LogLevel::INFO, "System Land");
        _land(tolerance);
    }

    void
    Copter::copter_Land(float ground_tolerance)
    {
        get_logger().write_show(LogLevel::INFO, "System Land");
        _land(ground_tolerance);
    }
#pragma endregion

#pragma region Setter
    int
    Copter::copter_set_speed(float speed_mps)
    {
        copter_speed = speed_mps;
        mavros_msgs::CommandLong speed_cmd;
        speed_cmd.request.command = 178;
        speed_cmd.request.param1 = 1; // ground speed type
        speed_cmd.request.param2 = speed_mps;
        speed_cmd.request.param3 = -1; // no throttle change
        speed_cmd.request.param4 = 0;  // absolute speed
        get_logger().write_show(LogLevel::INFO, "Set Speed : %.2f m/s", speed_mps);
        if (command_client.call(speed_cmd))
            return 0;
        get_logger().write_show(LogLevel::ERROR, "Change Speed Failed : %d",
                                speed_cmd.response.success);
        return -1;
    }

    void
    Copter::copter_set_ekf_source(EKFSource source)
    {
        mavros_msgs::RCIn rc;
        uint16_t data = 1000;
        switch (source)
        {
        case EKFSource::GPS_BARO:
            data = 1000;
            get_logger().write_show(LogLevel::INFO, "EKF Source : GPS-Baro [%d]",
                                    data);
            break;
        case EKFSource::GPS_GY:
            data = 1500;
            get_logger().write_show(LogLevel::INFO, "EKF Source : GPS-GY [%d]",
                                    data);
            break;
        case EKFSource::T265_GY:
            data = 2000;
            get_logger().write_show(LogLevel::INFO, "EKF Source : T265-GY [%d]",
                                    data);
            break;
        }

        rc.rssi = 0;
        rc.channels = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                       UINT16_MAX, data, UINT16_MAX, UINT16_MAX};
        cmd_rc_pub.publish(rc);
    }

    void
    Copter::copter_realign_viso() const
    {
        uint16_t rc7_pwm = 1000;
        mavros_msgs::RCIn rc1;
        rc1.rssi = 0;
        rc7_pwm = 2000;
        rc1.channels = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                        UINT16_MAX, UINT16_MAX, rc7_pwm, UINT16_MAX};
        cmd_rc_pub.publish(rc1);

        // Set back to low
        mavros_msgs::RCIn rc2;
        rc2.rssi = 0;
        rc7_pwm = 1000;
        rc2.channels = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                        UINT16_MAX, UINT16_MAX, rc7_pwm, UINT16_MAX};
        cmd_rc_pub.publish(rc2);
        get_logger().write_show(LogLevel::INFO, "Realign Viso");
    }

    // -7.276604973504977, 112.79385479212844
    void
    Copter::copter_set_ekf_origin(float lat, float lnt, float alt)
    {
        geographic_msgs::GeoPoseStamped origin;

        origin.pose.position.latitude = lat;
        origin.pose.position.longitude = lnt;
        origin.pose.position.altitude = alt;
        gps_pos_pub.publish(origin);
        get_logger().write_show(LogLevel::INFO, "Set Origin EKF on PENS");
    }

    int
    Copter::copter_set_home(float lat, float lnt, float alt)
    {
        mavros_msgs::CommandLong home_cmd;
        home_cmd.request.command = 179;
        home_cmd.request.param1 = 0;
        home_cmd.request.param2 = 0;
        home_cmd.request.param3 = 0;
        home_cmd.request.param4 = 0;
        home_cmd.request.param5 = lat;
        home_cmd.request.param6 = lnt;
        home_cmd.request.param7 = alt;
        get_logger().write_show(LogLevel::INFO, "Set home : %f, %f at %f m", lat,
                                lnt, alt);

        if (!command_client.call(home_cmd))
        {
            get_logger().write_show(LogLevel::ERROR, "Change home Failed %d",
                                    home_cmd.response.success);
            get_logger().write_show(LogLevel::ERROR, "Change home result was %d",
                                    home_cmd.response.result);
            return -1;
        }
        return 0;
    }

    void
    Copter::copter_set_rc(int channel, int pwm)
    {
        mavros_msgs::OverrideRCIn rc_override;
        for (int i = 0; i < 18; i++)
        {
            if (i == channel)
                rc_override.channels[i] = pwm;
            else
                rc_override.channels[i] = UINT16_MAX;
        }
        cmd_rc_override_pub.publish(rc_override);
    }

    int
    Copter::copter_set_mode(FlightMode mode)
    {
        std::string mode_str = "";
        switch (mode)
        {
        case FlightMode::LAND:
            mode_str = "LAND";
            break;
        case FlightMode::GUIDED:
            mode_str = "GUIDED";
            break;
        case FlightMode::AUTO:
            mode_str = "AUTO";
            break;
        case FlightMode::RTL:
            mode_str = "RTL";
            break;
        default:
            break;
        }

        mavros_msgs::SetMode srv_setMode;
        // srv_setMode.request.base_mode = 0;
        srv_setMode.request.custom_mode = mode_str.c_str();
        if (set_mode_client.call(srv_setMode) && srv_setMode.response.mode_sent)
        {
            get_logger().write_show(LogLevel::INFO, "Set Mode %s",
                                    mode_str.c_str());
            return 1;
        }
        else
        {
            get_logger().write_show(LogLevel::ERROR, "Failed Set Mode");
            return 0;
        }
    }
#pragma endregion

#pragma region Check
    bool
    Copter::copter_is_reached(WayPoint dest, float tolerance) const
    {
        geometry_msgs::Point cur_pos = _get_hexa_point();
        float _yaw_value = copter_get_yaw();
        float _yaw_diff = _yaw_value - dest.yaw;
        if (std::fabs(dest.yaw) > 175.0f || std::fabs(dest.yaw) < 5.0f)
            _yaw_diff = std::fabs(_yaw_value) - std::fabs(dest.yaw);
        return (std::fabs(cur_pos.x - dest.x) < tolerance && std::fabs(cur_pos.y - dest.y) < tolerance && std::fabs(_yaw_diff) < 5.0f);
    }

    bool
    Copter::copter_check_alt(float dist_alt, float tolerance) const
    {
        float _curr_alt = pose_data_local.pose.position.z;
        if (_curr_alt > dist_alt - tolerance && _curr_alt < dist_alt + tolerance)
        {
            get_logger().write_show(LogLevel::INFO,
                                    "Reached %.2f m target on %.2f m", dist_alt,
                                    _curr_alt);
            return true;
        }
        return false;
    }
#pragma endregion

#pragma region Tools
    void
    Copter::print_wp(std::string header, WayPoint &wp) const
    {
        std::vector<ListItem<float>> it;
        it.push_back({"x\t", wp.x, "m"});
        it.push_back({"y\t", wp.y, "m"});
        it.push_back({"z\t", wp.z, "m"});
        it.push_back({"yaw", wp.yaw, "m"});
        get_logger().list_show(header, it);
    }

    WayPoint
    Copter::copter_calc_transition(WayPoint start_point, WayPoint stop_point,
                                   float copter_deg, float copter_alt) const
    {
        WayPoint _wp;
        double dist_AB = sqrt(pow(stop_point.x - start_point.x, 2) + pow(start_point.y - stop_point.y, 2));

        double b_angle = atan2(stop_point.y - start_point.y, stop_point.x - start_point.x);

        double delta_AB = b_angle * 180.0 / M_PI - copter_deg;

        double distance = dist_AB * std::cos(delta_AB * M_PI / 180.0);
        double radians = copter_deg * M_PI / 180.0;

        double x_target = start_point.x + distance * cos(radians);
        double y_target = start_point.y + distance * sin(radians);

        _wp.x = x_target;
        _wp.y = y_target;
        _wp.z = copter_alt;
        _wp.yaw = copter_deg;
        return _wp;
    }

    void Copter::copter_Go_RTL(float alt, float tolerance)
    {
        if (alt >= 0.f)
            takeoff_wp.z = alt;
        copter_Go(takeoff_wp, true, "Go to RTL");

        WayPoint _wp;
        while (ros::ok())
        {
            copter_get_position(_wp);
            ros::spinOnce();
            if (_wp.x > takeoff_wp.x - tolerance && _wp.x < takeoff_wp.x + tolerance &&
                _wp.y > takeoff_wp.y - tolerance && _wp.y < takeoff_wp.y + tolerance &&
                _wp.z > takeoff_wp.z - tolerance && _wp.z < takeoff_wp.z + tolerance)
                break;
            ros::spinOnce();
            ros::Duration(0.2).sleep();
        }

        copter_Land(tolerance);
    }

#pragma endregion

    Copter::~Copter()
    {
        if (status == Status::Flying || status == Status::Takeoff)
            copter_Land(.5f);
        get_logger().finish();
        traj_logger.finish(false);
    }
} // namespace EMIRO
