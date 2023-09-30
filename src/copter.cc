#include <copter.hpp>

namespace EMIRO {
    void Copter::init(ros::NodeHandle *nh)
    {
        if(!is_init_pubs_subs)
        {
            copter_logger.init("Copter", EMIRO::FileType::CSV);
            //ROS Service Client
            command_client = (*nh).serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
            set_mode_client = (*nh).serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
            arming_client = (*nh).serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            takeoff_client = (*nh).serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
            land_client = (*nh).serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

            // RC (Remote Control) Publisher
            cmd_rc_pub = (*nh).advertise<mavros_msgs::RCIn>("/mavros/rc/in", 2);

            cmd_rc_override_pub = (*nh).advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);

            //ROS Subscriber/Publisher
            cmd_pos_pub = (*nh).advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 20);
            gps_pos_pub = (*nh).advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
            state_sub = (*nh).subscribe<mavros_msgs::State>("/mavros/state", 10, 
                [this](const mavros_msgs::State::ConstPtr& msg) {
                        this->state_cb(msg);
                    });
            cmd_pos_sub_local = (*nh).subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 2, 
                [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    this->pose_cb_local(msg);
                });

            cmd_vel_pub = (*nh).advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
            copter_logger.write_show(LogLevel::INFO, "Publisher and Subscriber initialized");
        }
        else
            copter_logger.write_show(LogLevel::WARNING, "Publisher and Subscriber already initialized");
    }

    bool Copter::FCUconnect(float timeout_s){
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;

        while (ros::ok() && !current_state_g.connected && _timeout_int)
        {
            _timeout_int--;
            ros::spinOnce();
            _timeout_rate.sleep();
        }
        if(current_state_g.connected)
            return true;
        return false;
    }

    bool Copter::FCUstart(float timeout_s)
    {
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;

        std::cout << "Waiting for GUIDED ";
        std::cout.flush();
        while(ros::ok() && current_state_g.mode != "GUIDED" && _timeout_int)
        {
            _timeout_int--;
            std::cout << ".";
            std::cout.flush();
            ros::spinOnce();
            _timeout_rate.sleep();
        }
        std::cout << '\n';

        if(current_state_g.mode == "GUIDED")
            return true;
        return false;
    }

    void Copter::init_frame(float timeout_s){
        ros::Rate _timeout_rate(20);
        int _timeout_int = timeout_s < 0.2f ? 4 : timeout_s * 20;
        if(!is_init_frame)
        {
            // Init copter frame
            ros::Rate _init_rate(5);
            int _counter = 20;
            std::cout << "Init Copter ";
            std::cout.flush();
            while(ros::ok() && _timeout_int)
            {
                std::cout << ".";
                std::cout.flush();
                _timeout_int--;
                ros::spinOnce();
                _timeout_rate.sleep();
            }
            std::cout << '\n';

            this->local_frame = this->get_current_position();
            if(local_frame.x == 0.000f && local_frame.y == 0.000f && local_frame.z == 0.000f)
                copter_logger.write_show(LogLevel::WARNING, "Local Frame looks unreasonable");
        }
        
    }

    Quaternion Copter::to_quaternion(float roll_rate, float pitch_rate,
                                        float yaw_rate) {
        float yaw = yaw_rate * (M_PI / 180);
        float pitch = pitch_rate * (M_PI / 180);
        float roll = roll_rate * (M_PI / 180);

        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);

        float qw = cy * cr * cp + sy * sr * sp;
        float qx = cy * sr * cp - sy * cr * sp;
        float qy = cy * cr * sp + sy * sr * cp;
        float qz = sy * cr * cp - cy * sr * sp;

        return {qw, qx, qy, qz};
    }

    geometry_msgs::Point Copter::enu_2_local(nav_msgs::Odometry current_pose_enu) {
        float x = current_pose_enu.pose.pose.position.x;
        float y = current_pose_enu.pose.pose.position.y;
        float z = current_pose_enu.pose.pose.position.z;
        float deg2rad = (M_PI / 180);
        geometry_msgs::Point current_pos_local;
        current_pos_local.x = x * cos((local_offset_g - 90) * deg2rad) -
                            y * sin((local_offset_g - 90) * deg2rad);
        current_pos_local.y = x * sin((local_offset_g - 90) * deg2rad) +
                            y * cos((local_offset_g - 90) * deg2rad);
        current_pos_local.z = z;
        return current_pos_local;
    }

    void Copter::pose_cb_local(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        pose_data_local = *msg;
        timestamp = pose_data_local.header.stamp;
    }

    void Copter::state_cb(const mavros_msgs::State::ConstPtr &msg) {
        current_state_g = *msg;
    }

    float Copter::get_yaw() {
        float w = pose_data_local.pose.orientation.w;
        float x = pose_data_local.pose.orientation.x;
        float y = pose_data_local.pose.orientation.y;
        float z = pose_data_local.pose.orientation.z;
        return 360.0f - (atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / M_PI);
    }

    int Copter::set_mode(CopterMode mode) {
        std::string mode_str = "";
        switch (mode) {
        case CopterMode::LAND:
            mode_str = "LAND";
            break;
        case CopterMode::GUIDED:
            mode_str = "GUIDED";
            break;
        case CopterMode::AUTO:
            mode_str = "AUTO";
            break;
        case CopterMode::RTL:
            mode_str = "RTL";
            break;
        default:
            break;
        }

        mavros_msgs::SetMode srv_setMode;
        // srv_setMode.request.base_mode = 0;
        srv_setMode.request.custom_mode = mode_str.c_str();
        if (set_mode_client.call(srv_setMode) && srv_setMode.response.mode_sent) {
            copter_logger.write_show(LogLevel::INFO, "Set Mode %s", mode_str.c_str());
            return 1;
        } else {
            
        ROS_ERROR("Failed Set Mode");
        return 0;
        }
    }

    geometry_msgs::Point Copter::get_hexa_point() {
        return pose_data_local.pose.position;
    }

    void Copter::go_to(geometry_msgs::Pose pose) {
        pose_stamped.header.stamp = timestamp;
        pose_stamped.pose = pose;

        cmd_pos_pub.publish(pose_stamped);
    }

    void Copter::goto_xyz_rpy(float x, float y, float z, float roll,
                                    float pitch, float yaw) {
        pose_data2.position.x = x;
        pose_data2.position.y = y;
        pose_data2.position.z = z;
        
        Quaternion _q = this->to_quaternion(roll, pitch, yaw + M_PI_2);

        pose_data2.orientation.x = _q.x;
        pose_data2.orientation.y = _q.y;
        pose_data2.orientation.z = _q.z;
        pose_data2.orientation.w = _q.w;

        go_to(pose_data2);
    }

    void Copter::set_vel(float vx, float vy, float vz, float avx, float avy,
                                float avz) {
        cmd_vel.linear.x = vx;
        cmd_vel.linear.y = vy;
        cmd_vel.linear.z = vz;

        cmd_vel.angular.x = avx;
        cmd_vel.angular.y = avy;
        cmd_vel.angular.z = avz;

        cmd_vel_pub.publish(cmd_vel);
    }

    bool Copter::Arming(){
        goto_xyz_rpy(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        for(int i=0; i<10; i++)
        {
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        std::cout << "Arming drone ";
        std::cout.flush();
        mavros_msgs::CommandBool arm_request;
        arm_request.request.value = true;
        while (!current_state_g.armed && !arm_request.response.success && ros::ok())
        {
            std::cout << ".";
            std::cout.flush();
            arming_client.call(arm_request);
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            ros::Duration(.1).sleep();
        }

        if(arm_request.response.success)
        {
            std::cout << " (OK)\n";
            copter_logger.write_show(LogLevel::INFO, "Drone armed successfully.") ; 
            return true;
        }
        std::cout << " (FAILED)\n";
        copter_logger.write_show(LogLevel::ERROR, "FAILED to arming drone. Code : %d", arm_request.response.success);
        return false;
    }

    int Copter::takeoff(float takeoff_alt)
    {
        goto_xyz_rpy(0,0,takeoff_alt,0,0,0);
        for(int i=0; i<10; i++)
        {
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        // arming
        ROS_INFO("Arming drone");
        mavros_msgs::CommandBool arm_request;
        arm_request.request.value = true;
        while (!current_state_g.armed && !arm_request.response.success && ros::ok())
        {
            ros::Duration(.1).sleep();
            arming_client.call(arm_request);
            cmd_pos_pub.publish(pose_stamped);
        }

        if(arm_request.response.success)
            ROS_INFO("Arming Successful");  
        else{
            ROS_ERROR("Arming FAILED : %d", arm_request.response.success);
            return -1;  
        }

        //request takeoff
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = takeoff_alt;
        if(takeoff_client.call(srv_takeoff))
            ROS_INFO("Success Takeoff");
        else{
            ROS_ERROR("Failed Takeoff");
            return -2;
        }

        return 0;
    }

    int Copter::just_takeoff(float takeoff_alt, float _yaw)
    {
        // Update location data
        ros::Rate _internal_rate(2);
        int _timer = 5;
        while(ros::ok() && _timer > 0)
        {
            _timer--;
            ros::spinOnce();
            _internal_rate.sleep();
        }

        // Get latest position data
        WayPoint _wp = get_current_position();
        goto_xyz_rpy(_wp.x, _wp.y, takeoff_alt, 0, 0, _yaw);
        for(int i=0; i<10; i++)
        {
        cmd_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        }

        // Arming drone
        ROS_INFO("Arming drone");
        mavros_msgs::CommandBool arm_request;
        arm_request.request.value = true;
        while (!current_state_g.armed && !arm_request.response.success && ros::ok())
        {
        ros::Duration(.1).sleep();
        arming_client.call(arm_request);
        cmd_pos_pub.publish(pose_stamped);
        }
        if(arm_request.response.success)
        ROS_INFO("Arming Successful");  
        else{
        ROS_ERROR("Arming FAILED : %d", arm_request.response.success);
        return -1;  
        }

        //Takeoff drone
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = takeoff_alt;
        if(takeoff_client.call(srv_takeoff))
        ROS_INFO("Success Takeoff");
        else{
        ROS_ERROR("Failed Takeoff");
        return -2;
        }

        return 0;
    }

    int Copter::takeoff2(WayPoint wp)
    {
        goto_xyz_rpy(wp.x, wp.y, wp.z, 0,0, wp.yaw);
        for(int i=0; i<100; i++)
        {
            cmd_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        // arming
        ROS_INFO("Arming drone");
        mavros_msgs::CommandBool arm_request;
        arm_request.request.value = true;
        while (!current_state_g.armed && !arm_request.response.success && ros::ok())
        {
            ros::Duration(.1).sleep();
            arming_client.call(arm_request);
            cmd_pos_pub.publish(pose_stamped);
        }
        if(arm_request.response.success)
        {
            ROS_INFO("Arming Successful");  
        }else{
            ROS_INFO("Arming FAILED : %d", arm_request.response.success);
            return -1;  
        }

        //request takeoff
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = wp.z;
        if(takeoff_client.call(srv_takeoff)){
            ROS_INFO("Success Takeoff");
        }else{
            ROS_ERROR("Takeoff FAILED");
            return -2;
        }

    return 0;
    }

    int Copter::land()
    {
        mavros_msgs::CommandTOL srv_land;
        if(land_client.call(srv_land) && srv_land.response.success)
        {
        ROS_INFO("Success Land");
        return 0;
        }else{
        ROS_ERROR("Land FAILED : %d", srv_land.response.success);
        return -1;
        }
    }

    int Copter::set_speed(float speed_mps)
    {
        copter_speed = speed_mps;
        mavros_msgs::CommandLong speed_cmd;
        speed_cmd.request.command = 178;
        speed_cmd.request.param1 = 1; // ground speed type 
        speed_cmd.request.param2 = speed_mps;
        speed_cmd.request.param3 = -1; // no throttle change
        speed_cmd.request.param4 = 0; // absolute speed
        ROS_INFO("Set Speed : %.2f m/s", speed_mps);
        if(command_client.call(speed_cmd))
        return 0;
        else{
        ROS_ERROR("Change speed command failed %d", speed_cmd.response.success);
        ROS_ERROR("Change speed result was %d ", speed_cmd.response.result);
        return -1;
        }
        return 0;
    }

    void Copter::set_ekf_source(EKF_Source source)
    {
        mavros_msgs::RCIn rc;
        uint16_t data =  1000;
        switch(source)
        {
        case EKF_Source::GPS_BARO:
        data = 1000;
        ROS_INFO("EKF Source : GPS-Baro (%d)", data);
        break;
        case EKF_Source::GPS_GY:
        data = 1500;
        ROS_INFO("EKF Source : GPS-GY (%d)", data);
        break;
        case EKF_Source::T265_GY:
        data = 2000;
        ROS_INFO("EKF Source : T265-GY (%d)", data);
        break;
        }

        rc.rssi = 0;
        rc.channels = {UINT16_MAX,
        UINT16_MAX, 
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX, 
        data, 
        UINT16_MAX, 
        UINT16_MAX};
        cmd_rc_pub.publish(rc);
    }

    void Copter::viso_align()
    {
        uint16_t rc7_pwm =  1000;
        mavros_msgs::RCIn rc1;
        rc1.rssi = 0;
        rc7_pwm = 2000;
        rc1.channels = {
        UINT16_MAX,
        UINT16_MAX, 
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX, 
        UINT16_MAX, 
        rc7_pwm, 
        UINT16_MAX};
        cmd_rc_pub.publish(rc1);

        // Set back to low
        mavros_msgs::RCIn rc2;
        rc2.rssi = 0;
        rc7_pwm = 1000;
        rc2.channels = {
        UINT16_MAX,
        UINT16_MAX, 
        UINT16_MAX,
        UINT16_MAX,
        UINT16_MAX, 
        UINT16_MAX, 
        rc7_pwm, 
        UINT16_MAX};
        cmd_rc_pub.publish(rc2);
        ROS_INFO("Realign viso");
    }

    // -7.276604973504977, 112.79385479212844
    void Copter::set_ekf_origin(float lat, float lnt, float alt) {
    geographic_msgs::GeoPoseStamped origin;

    origin.pose.position.latitude = lat;
    origin.pose.position.longitude = lnt;
    origin.pose.position.altitude = alt;
    gps_pos_pub.publish(origin);
    ROS_INFO("Set Origin EKF on PENS");
    }

    int Copter::set_home(float lat, float lnt, float alt)
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
        ROS_INFO("Set home : %f, %f at %f m", lat, lnt, alt);
        if(!command_client.call(home_cmd))
        {
        ROS_ERROR("Change home FAILED %d", home_cmd.response.success);
        ROS_ERROR("Change home result was %d ", home_cmd.response.result);
        return -1;
        }
        return 0;
    }

    bool Copter::is_reached(WayPoint dest, float tolerance) {
        geometry_msgs::Point cur_pos = get_hexa_point();
        return (std::fabs(std::fabs(cur_pos.x) - std::fabs(dest.x)) < tolerance &&
                std::fabs(std::fabs(cur_pos.y) - std::fabs(dest.y)) < tolerance &&
                std::fabs(std::fabs(cur_pos.z) - std::fabs(dest.z)) < tolerance);
    }

    void Copter::Go(WayPoint& wp)
    {
        std::cout << "\n\033[32m\033[1mGo to :\033[22m" << wp << "\033[0m" << std::endl;
        goto_xyz_rpy(wp.x, wp.y, wp.z, 0, 0, wp.yaw);
    }

    void Copter::Go2(WayPoint& wp)
    {
        goto_xyz_rpy(wp.x, wp.y, wp.z, 0, 0, wp.yaw);
    }

    void Copter::Go_Out(WayPoint& wp)
    {
        std::cout << "\n\033[32m\033[1mGo to :\033[22m" << wp << "\033[0m" << std::endl;
        goto_xyz_rpy(wp.x, wp.y, wp.z - 3.0f, 0, 0, wp.yaw);
    }

    /*float Copter::alt_correction(const float sensor_alt, float target_alt, float tolerance)
    {
        float vision_alt = pose_data_local.pose.position.z;

        if (sensor_alt < copter_param.RNGFND1_MIN) {
            copter_logger.write_show("Sensor value below minimum value. (REJECTED)");
            return vision_alt;
        }

        float alt_meter = sensor_alt / 100.0f;
        alt_meter -= 0.12f; // Remove body altitude

        // Check altitude correction
        float abs_diff = std::fabs(target_alt - alt_meter);
        if(abs_diff > tolerance && abs_diff < 1.0f){
            float correction_alt = target_alt - alt_meter;
            float high = vision_alt + correction_alt;
            Go(wp);
            return wp.z;
        }
        else{
            ROS_WARN("Danger altitude. Failed correction : %lf", abs_diff);
            return initial_altitude;
        }
    }*/

    Mode Copter::get_current_mission() { return this->misi_mode; }

    void Copter::Go_Land(WayPoint wp) {
        ROS_INFO("System Go to Land");
        Go(wp);
        ros::Duration(4).sleep();
        ROS_INFO("System Land");
        land();
    }

    void Copter::rc_override(int channel, int pwm)
    {
        mavros_msgs::OverrideRCIn rc_override;
        for (int i = 0; i < 18; i++)
        {
        if(i == channel)
            rc_override.channels[i] = pwm;
        else
            rc_override.channels[i] = UINT16_MAX;
        }
        cmd_rc_override_pub.publish(rc_override);
    }

    void Copter::Land() {
        ROS_INFO("System Land");
        land();
    }

    geometry_msgs::Pose Copter::get_local_pose() {
        return pose_data_local.pose;
    }

    EMIRO::WayPoint Copter::get_current_position(){
        float x_ = pose_data_local.pose.position.x;
        float y_ = pose_data_local.pose.position.y;
        float z_ = pose_data_local.pose.position.z;
        EMIRO::WayPoint _temp_ = {x_, y_, z_, get_yaw()};
        return _temp_;
    }

    WayPoint Copter::calc_transition(WayPoint start_point, WayPoint stop_point, float copter_deg, float copter_alt)
    {
        WayPoint _wp;
        double dist_AB = sqrt(pow(stop_point.x - start_point.x, 2) + pow(start_point.y - stop_point.y, 2));
        
        double b_angle = atan2(stop_point.y - start_point.y, stop_point.x - start_point.x);
        
        double delta_AB = b_angle * 180.0 / M_PI - copter_deg;
        
        double distance = dist_AB * std::cos(delta_AB*M_PI/180.0);
        double radians = copter_deg * M_PI / 180.0;
        
        double x_target = start_point.x + distance * cos(radians);
        double y_target = start_point.y + distance * sin(radians);
        
        _wp.x = x_target;
        _wp.y = y_target;
        _wp.z = copter_alt;
        _wp.yaw = copter_deg;
        return _wp;
    }

    bool Copter::check_alt(float dist_alt, float tolerance)
    {   
        float _curr_alt = pose_data_local.pose.position.z;
        if(_curr_alt > dist_alt - tolerance && _curr_alt < dist_alt + tolerance)
        {
            ROS_INFO("Reached %.2f m target on %.2f m", dist_alt, _curr_alt);
            return true;
        }
        return false;
    }

    float Copter::get_alt()
    {
        return pose_data_local.pose.position.z;
    }

    void Copter::get_position(WayPoint &wp, int counter)
    {
        ros::Rate in_rate(5);
        counter = counter < 2 ? 2 : counter;
        while(ros::ok() && counter)
        {
            counter--;
            ros::spinOnce();
            in_rate.sleep();
        }
        wp.x = pose_data_local.pose.position.x;
        wp.y = pose_data_local.pose.position.y;
        wp.z = pose_data_local.pose.position.z;
    }
}