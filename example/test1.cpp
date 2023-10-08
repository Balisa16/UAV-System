// rosrun emiro test1 1 0 1 0.5 5 0.3 8 1 1 1 1


#include <convert.hpp>
#include <enum.hpp>
#include <copter.hpp>
#include <servo.hpp>
#include <lidar.hpp>
#include <GPSnav.hpp>
#include <rangefinder.hpp>

#include <iostream>
#include <cstring>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <memory>

/*int main()
{
    std::shared_ptr<EMIRO::Copter> copter = std::make_shared<EMIRO::Copter>();
    return 0;
}*/

namespace EMIRO{
    class Misi2
    {
    private:
        ros::NodeHandle nh;
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        EMIRO::Servo ser;
        EMIRO::Nav_Convert converter;
        EMIRO::GlobalNav global_nav;
        EMIRO::Lidar lidar_dev;
        EMIRO::RangeFinder rangefinder;

        // Constant param
        const Color_Range _drop_color = {{10, 0, 0}, {180, 255, 255}};
        const Color_Range _load_color = {{81, 110, 110}, {180, 255, 255}};

        // Misi Variable
        bool left_mission;
        int camera_idx;
        bool real_prog;
        EMIRO::WayPoint _drop_pos, _load_pos, _start_pos, _finish_pos;
        Indoor_State _in_state, _out_state;

        float _multiply, _load_alt = 0.3f, _indoor_alt = 0.8f, _outdoor_alt = 1.2f, _pos_tolerance = 0.2f;
        int _timeout;
        bool _timeout_en = true;
        bool take_load = true;
        float _gps_yaw, _indoor_speed = 0.8f, _outdoor_speed = 6.0f;
        float left_right_min = 1.0f;
        float interchange_min = 1.30f;
        float current_yaw;

        float _load_lidar_front, _load_lidar_right, _load_lidar_left;
        float _indoor_speed_2;

        // Outdoor variable
        float x_range = 50.0f;
        float y_range = 100.0f;
        float mul_right = 1.0f;
        float mul_left = 1.0f;
        float mul_front = 1.0f;
        float mul_back = 1.0;

        EMIRO::WayPoint lock_pos(int min_counter = 1, const char *title = "Press y to lock position", bool nav_change = false);

        void lidar_lock(float& front, float& left, float& right);

        void resume();

        bool go_forward(Axis& axis, float diff_yaw);

        bool go_left(Axis& axis, float diff_yaw);

        bool go_right(Axis& axis, float diff_yaw);

        void yaw_correction();

        void alt_correction(float des_alt, float tolerance = 0.04);

        void match_load();

        float get_diff_yaw();

        void calc_axis(Axis& axis);

        void go_vel(LinearSpeed &ls, float speed_limit = 0.5f);

        void stay(Axis& axis);

        void alt_correction_outdoor(float des_alt, float tolerance = 2.0f, float wait_delay = 1.0f);

        // Template variable
        LinearSpeed linear_temp;

    public:

        Misi2(int argc, char **argv);
        
        void Indoor();

        bool is_start();

        void pre_arm();

        void Outdoor();

        void Land();

        ~Misi2();
    };

    inline void Misi2::alt_correction_outdoor(float des_alt, float tolerance, float wait_delay)
    {
        ros::Rate _correction_rate(2);
        float _cur_alt;
        ros::Duration(wait_delay).sleep();
        std::cout << "Altitude correction ";
        while(ros::ok())
        {
            std::cout << ".";
            std::cout.flush();
            _cur_alt = copter->get_alt();
            if(_cur_alt > des_alt + tolerance)
                copter->set_vel(0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f);
            else if(_cur_alt < des_alt - tolerance)
                copter->set_vel(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
            else
                break;
            ros::spinOnce();
            _correction_rate.sleep();
        }
        copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        std::cout << " (OK)" << std::endl;
    }


    inline void Misi2::stay(Axis& axis)
    {
        linear_temp = {0.0000f, 0.0000f};
        if(axis.left < left_right_min)
            linear_temp.linear_x = -_indoor_speed_2;
        if(axis.right < left_right_min)
            linear_temp.linear_x = _indoor_speed_2;
        go_vel(linear_temp, _indoor_speed_2);
    }

    inline float Misi2::get_diff_yaw()
    {
        current_yaw = copter->get_yaw();
        float temp_yaw  =  current_yaw - _gps_yaw;
        if(temp_yaw > 180)
            temp_yaw = -(_gps_yaw + (360 - current_yaw));
        else if(temp_yaw < -180)
            temp_yaw = (360 - _gps_yaw) + current_yaw;
        return temp_yaw;
    }

    inline void Misi2::calc_axis(Axis& axis)
    {
        current_yaw = copter->get_yaw();
        float _diff_degree = std::abs(get_diff_yaw());

        if(_diff_degree < 90.0)
        {
            if(axis.front < 20.0f)
                axis.front *= std::cos(_diff_degree*M_PI/180);

            if(axis.left < 20.0f)
                axis.left *= std::cos(_diff_degree*M_PI/180);

            if(axis.right < 20.0f)
                axis.right *= std::cos(_diff_degree*M_PI/180);
            
            if(axis.back < 20.0f)
                axis.back *= std::cos(_diff_degree*M_PI/180);
        }
    }

    inline void Misi2::yaw_correction()
    {
        copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        ros::Rate _internal_rate(5);
        float yaw_thresh = 15.0f;
        logger->wait("Yaw Correction");
        while(ros::ok())
        {
            float _diff_deg = get_diff_yaw();
            if(std::abs(_diff_deg) < yaw_thresh)
                break;
            
            float speed2calc = _diff_deg;
            if(_diff_deg > 25.0f)
                speed2calc = 40.0f;
            else if(_diff_deg < -25.0f)
                speed2calc = -40.0f;
            float _speed_yaw = (speed2calc/2.0f)*M_PI/180;
            copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, _speed_yaw);
            

            ros::spinOnce();
            _internal_rate.sleep();
        }
        logger->wait_success();
        copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        ros::Duration(0.5f).sleep();
    }

    inline void Misi2::alt_correction(float des_alt, float tolerance)
    {
        copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        ros::Rate _internal_rate(3);
        logger->wait("Altitude Correction");
        while(ros::ok())
        {
            float _curr_alt = this->rangefinder.get_range();
            if(_curr_alt < 2.0f && _curr_alt > 0.0f)
            {
                if(_curr_alt > des_alt + tolerance)
                    copter->set_vel(0.0f, 0.0f, -0.2f, 0.0f, 0.0f, 0.0f);
                else if(_curr_alt < des_alt - tolerance)
                    copter->set_vel(0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
                else
                {
                    logger->wait_success();
                    break;
                }
            }
            else
            {
                logger->wait_failed();
                break;
            }
        
            ros::spinOnce();
            _internal_rate.sleep();
        }
        copter->set_vel(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        ros::Duration(0.5f).sleep();
    }


    inline EMIRO::WayPoint Misi2::lock_pos(int min_counter, const char *title, bool nav_change)
    {
        try_flag:
        const char* new_line = " Press y to lock.\n";  
        char result[strlen(title) + strlen(new_line)];
        strcpy(result, title);
        strcat(result, new_line);
        
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
        keypad(stdscr, TRUE);

        printw(result);
        refresh();

        if(nav_change)
            copter->set_ekf_source(EMIRO::EKF_Source::GPS_BARO);

        ros::Duration(1).sleep();
        ros::Rate in_rate(5);

        EMIRO::WayPoint temp_wp;
        std::cout << std::fixed << std::setprecision(2);
        bool _btn_trigger = false;
        int _btn_trigger_counter = 1;
        
        while(ros::ok())
        {
            temp_wp = copter->get_current_position();
            std::cout << temp_wp.x << " m    " << temp_wp.y << " m    " << temp_wp.z << "  m       \r  ";
            std::cout.flush();
            int ch = getch();
            if(ch == 'y' || ch == 'Y' || _btn_trigger_counter < 0)
            {
                _btn_trigger = true;
                _btn_trigger_counter = min_counter;
                if(min_counter < 0)
                    break;
                else
                    std::cout << "Please wait ..." << std::endl;
            }
            if(_btn_trigger)
                _btn_trigger_counter--;
            min_counter--;
            ros::spinOnce();
            in_rate.sleep();
        }
        endwin();
        std::cout << "Last " << temp_wp << std::endl;
        std::cout << "Accept ? (y/n) " << std::endl;
        char choice;
        std::cin >> choice;
        if(!(choice == 'y' || choice == 'Y'))
            goto try_flag;
        if(nav_change)
            copter->set_ekf_source(EMIRO::EKF_Source::T265_GY);
        return temp_wp;
    }

    inline void Misi2::resume()
    {
        std::cout << "\n\033[31m\033[1mMission Parameter :\033[0m" << std::endl;
        std::cout << "\033[32mMission Type\t: \033[0m" << (this->left_mission ? "Left" : "Right") << std::endl;
        std::cout << "\033[32mStart Position\t: \033[0m" << this->_start_pos << std::endl;
        std::cout << "\033[32mLoad Lidar Front : \033[0m" << this-> _load_lidar_front << " m" << std::endl;
        std::cout << "\033[32mLoad Lidar Right : \033[0m" << this-> _load_lidar_right << " m" << std::endl;
        std::cout << "\033[32mCamera Index\t: \033[0m" << this->camera_idx << std::endl;
        std::cout << "\033[32mIndoor Speed\t: \033[0m" << this->_indoor_speed * 3.6 << " km/h" << std::endl;
        std::cout << "\033[32mOutdoor Speed\t: \033[0m" << this->_outdoor_speed * 3.6 << " km/h" << std::endl;
        std::cout << "\033[32mLeft \t: \033[0m" << this->mul_left << " x" << std::endl;
        std::cout << "\033[32mRight \t: \033[0m" << this->mul_right << " x" << std::endl;
        std::cout << "\033[32mFront \t: \033[0m" << this->mul_front << " x" << std::endl;
        std::cout << "\033[32mBack \t: \033[0m" << this->mul_back << " x" << std::endl << std::endl;
    }

    Misi2::Misi2(int argc, char **argv)
    {

        // Init Variable
        this->left_mission = atoi(argv[1]) > 0;
        _multiply = this->left_mission ? -1.0f : 1.0f;
        this->camera_idx = atoi(argv[2]);
        this->real_prog = atoi(argv[3]) > 0 ? true : false;
        _indoor_alt = atof(argv[4]);
        _outdoor_alt = atof(argv[5]);
        _indoor_speed = atof(argv[6]);
        _outdoor_speed = atof(argv[7]);

        mul_left = atof(argv[8]);
        mul_right = atof(argv[9]);
        mul_front = atof(argv[10]);
        mul_back = atof(argv[11]);
        _indoor_speed_2 = _indoor_speed*0.75;

        logger = std::make_shared<Logger>();
        copter = std::make_shared<Copter>();
        logger->init("Copter", FileType::CSV);
        logger->start(true);
        copter->init(&this->nh, logger);

        // Initialize lidar
        this->lidar_dev.init(copter, logger);
        // this->lidar_dev.start(&nh, LidarType::S1);
        this->lidar_dev.start(&nh, LidarType::Simulator);

        
        logger->wait("Waiting Lidar");
        ros::Rate _lidar_wait(2);
        while(ros::ok() && lidar_dev.check() != LidarStatus::Run)
        {
            ros::spinOnce();
            _lidar_wait.sleep();
        }
        logger->wait_success();

        // Get lidar pos
        lidar_lock(_load_lidar_front, _load_lidar_left, _load_lidar_right);
        
        // Lock start position
        _start_pos = lock_pos(10, "Start position.", false);
        _gps_yaw = _start_pos.yaw;
        converter.init(_gps_yaw);
        global_nav.init(_gps_yaw);

        _finish_pos = _start_pos;
        _finish_pos.x += (left_mission ? -3.15 : 3.15);

        // Initialize servo
        this->ser.init(copter, logger);
        this->ser.custom_pwm(9, 1400);
        this->ser.custom_pwm(10, 1500);

        resume();
    }

    inline void Misi2::go_vel(LinearSpeed &ls, float speed_limit)
    {
        LinearSpeed _speed_go = global_nav.convert(ls, speed_limit); 
        copter->set_vel(_speed_go.linear_x, _speed_go.linear_y, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    inline void Misi2::match_load()
    {
        logger->write_show(LogLevel::INFO, "Match Payload");
        ros::Rate rr(8);
        float match_speed = 0.20f;
        while(ros::ok())
        {
            current_yaw = copter->get_yaw();
            float _diff_yaw = get_diff_yaw();
            linear_temp = {0.0f, 0.0f};
            if(left_mission)
            {
                float _right_scan = lidar_dev.get_right(1);
                _right_scan *= std::cos(_diff_yaw*M_PI/180.0f);
                if(_right_scan < 10.0f)
                {
                    if(_right_scan > _load_lidar_right + 0.1f)
                        linear_temp.linear_x = -match_speed;
                    else if(_right_scan < _load_lidar_right)
                        linear_temp.linear_x = match_speed;
                    else
                    {
                        go_vel(linear_temp, match_speed);
                        break;
                    }
                }
            }
            else
            {
                float _left_scan = lidar_dev.get_left(1);
                _left_scan *= std::cos(_diff_yaw*M_PI/180.0f);
                if(_left_scan < 10.0f)
                {
                    if(_left_scan > _load_lidar_left + 0.1f)
                        linear_temp.linear_x = match_speed;
                    else if(_left_scan < _load_lidar_left - 0.1f)
                        linear_temp.linear_x = -match_speed;
                    else
                    {
                        go_vel(linear_temp, match_speed);
                        break;
                    }
                }
            }
            go_vel(linear_temp, match_speed);
            ros::spinOnce();
            rr.sleep();
        }
        ros::Duration(1).sleep();
    }

    inline void Misi2::Indoor()
    {
        ros::Rate rr(5);
        copter->takeoff(_indoor_alt);
        ros::Duration(5).sleep();

        LinearSpeed _temp_linear = {0.0f, 0.0f};

        Axis _axis_temp;
        float _front_temp;
        float _temp1_val;
        bool _interrupt_switch = false;

        Movement _indoor_movement = Movement::Forward;
        int hover_inc = 50;
        float yaw_maks = 15.0f;
        while (ros::ok() && !_interrupt_switch)
        {
            current_yaw = copter->get_yaw();
            float _diff_yaw = get_diff_yaw();
            lidar_dev.axis(_axis_temp);
            calc_axis(_axis_temp);

            if(std::abs(_diff_yaw) > yaw_maks && _indoor_movement != Movement::TakeLoad)
                yaw_correction();
            else{
                switch (_indoor_movement)
                {
                case Movement::Forward:

                    if(!go_forward(_axis_temp, _diff_yaw))
                    {
                        if(left_mission)
                            logger->write_show(LogLevel::INFO, "Go to Left");
                        else
                            logger->write_show(LogLevel::INFO, "Go to Right");
                        _indoor_movement = Movement::Left_Right;
                    }
                    break;

                case Movement::TakeLoad:
                    stay(_axis_temp);
                    hover_inc--;
                    if(hover_inc < 0)
                    {
                        take_load = true;
                        alt_correction(0.58f);
                        yaw_maks = 20.0f;
                        logger->write_show(LogLevel::INFO, "Go to Forward again");
                        _indoor_movement = Movement::Forward;
                    }
                    break;
                
                case Movement::Left_Right:
                    if(left_mission)
                    {
                        if(!go_left(_axis_temp, _diff_yaw))
                        {
                            logger->write_show(LogLevel::INFO, "Droping Payload");
                            _indoor_movement = Movement::DropLoad;
                        }
                    }
                    else if(!go_right(_axis_temp, _diff_yaw))
                    {
                        logger->write_show(LogLevel::INFO, "Droping Payload");
                        _indoor_movement = Movement::DropLoad;
                    }
                    break;
                
                case Movement::DropLoad:
                    _interrupt_switch = true;
                    break;

                default:
                    ROS_ERROR("Unknown Movement State");
                    break;
                }
            }
            ros::spinOnce();
            rr.sleep();
        }
        if(left_mission)
            linear_temp = {_indoor_speed, 0.0f};
        else
            linear_temp = {-_indoor_speed, 0.0f};

        LinearSpeed _speed_go = global_nav.convert(linear_temp, _indoor_speed); 
        copter->set_vel(0.0f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f);
        ros::Duration(1.0f).sleep();
        copter->set_vel(_speed_go.linear_x, _speed_go.linear_y, 0.0f, 0.0f, 0.0f, 0.0f);
        ros::Duration(2.5f).sleep();

        yaw_correction();
        _temp_linear = {0.0f, 0.0f};
        go_vel(_temp_linear, _indoor_alt);
        this->ser.custom_pwm(9, 1800);
        ros::Duration(3).sleep();
        this->ser.custom_pwm(9, 1450);

        logger->write_show(LogLevel::INFO, "Finished Indoor Mission");
    }

    inline bool Misi2::is_start()
    {
        pre_arm();
        char ch;
        std::cout << "Start Mission ? (y/n)" << std::endl; 
        std::cin >> ch;
        if(ch == 'y' || ch == 'Y')
            return true;
        return false;
    }

    inline void Misi2::pre_arm()
    {
        copter->FCUconnect(2.0f);
        copter->FCUstart(1.0f);
        copter->init_frame();
    }

    inline void Misi2::Land()
    {
        copter->Land();
    }

    inline void Misi2::lidar_lock(float& front, float& left, float& right)
    {
        try_flag:
        front = lidar_dev.get_front(1);
        left = lidar_dev.get_left(1);
        right = lidar_dev.get_right(1);

        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
        keypad(stdscr, TRUE);

        printw("Lock lidar scan ? (y/n)\n");
        refresh();

        ros::Rate in_rate(5);
        std::cout << std::fixed << std::setprecision(2);

        while(ros::ok())
        {
            front = lidar_dev.get_front(1);
            left = lidar_dev.get_left(1);
            right = lidar_dev.get_right(1);
            std::cout << "F : " << front << " m,   L : " << left << " m,   R : " << right << " m             \r  ";
            std::cout.flush();
            int ch = getch();
            if(ch == 'y' || ch == 'Y')
                break;
            ros::spinOnce();
            in_rate.sleep();
        }
        endwin();
        std::cout << std::fixed << std::setprecision(2) << "Lidar :\n\tFront : " << front << " m\n\tLeft : " << left  << " m\n\tRight : " << right << " m\n";
        std::cout << "Accept ? (y/n) " << std::endl;
        char choice;
        std::cin >> choice;
        if(!(choice == 'y' || choice == 'Y'))
            goto try_flag;
    }
    

    inline bool Misi2::go_forward(Axis& axis, float diff_yaw)
    {
        if(take_load && axis.front < interchange_min)
        {
            if(left_mission && axis.left > 2.2f)
                return false;
            else if(!left_mission && axis.right > 2.2f)
                return false;
        }

        linear_temp = {0.0000f, 0.0000f};
        linear_temp.linear_y = _indoor_speed;
        bool _is_left_right = false;
        if(axis.left < left_right_min)
        {
            _is_left_right = true;
            linear_temp.linear_x = -_indoor_speed_2;
        }
        if(axis.right < left_right_min)
        {
            _is_left_right = true;
            linear_temp.linear_x = _indoor_speed_2;
        }

        if(!_is_left_right)
        {
            // std::cout << "No Object" << std::endl;
            linear_temp.linear_y = _indoor_speed * std::cos(((int)diff_yaw%360)*M_PI/180);
            linear_temp.linear_x = _indoor_speed * std::sin(((int)diff_yaw%360)*M_PI/180);
        }

        go_vel(linear_temp, _indoor_speed);
        return true;
    }

    inline bool Misi2::go_left(Axis& axis, float diff_yaw)
    {
        if(axis.front > 3.0f && axis.back > 3.0f)
            return false;

        linear_temp = {0.0000f, 0.0000f};

        linear_temp.linear_x = _indoor_speed;
        bool _is_front_back = false;
        if(axis.front < left_right_min)
        {
            _is_front_back = true;
            linear_temp.linear_y = -_indoor_speed_2;
        }
        if(axis.back < left_right_min)
        {
            _is_front_back = true;
            linear_temp.linear_y = _indoor_speed_2;
        }
        
        if(!_is_front_back)
        {
            // std::cout << "No Object" << std::endl;
            linear_temp.linear_x = _indoor_speed * std::cos((((int)diff_yaw)%360)*M_PI/180);
            linear_temp.linear_y = _indoor_speed * std::sin((((int)diff_yaw)%360)*M_PI/180);
        }

        go_vel(linear_temp, _indoor_speed);
        return true;
    }
    
    inline bool Misi2::go_right(Axis& axis, float diff_yaw)
    {
        if(axis.front > 3.0 && axis.back > 3.0)
            return false;

        linear_temp = {0.0000f, 0.0000f};
        linear_temp.linear_x = -_indoor_speed;
        bool _is_front_back = false;
        if(axis.front < left_right_min)
        {
            _is_front_back = true;
            linear_temp.linear_y = -_indoor_speed_2;
        }
        if(axis.back < left_right_min)
        {
            _is_front_back = true;
            linear_temp.linear_y = _indoor_speed_2;
        }

        if(!_is_front_back)
        {
            linear_temp.linear_x = -_indoor_speed * std::cos((((int)diff_yaw)%360)*M_PI/180);
            linear_temp.linear_y = _indoor_speed * std::sin((((int)diff_yaw)%360)*M_PI/180);
        }

        go_vel(linear_temp, _indoor_speed);
        return true;
    }

    inline void Misi2::Outdoor()
    {
        WayPoint2 curr_pos = {_start_pos.x, _start_pos.y, _outdoor_alt, 360.0f - _start_pos.yaw, _outdoor_speed};
        WayPoint2 _temp_pos = curr_pos;

        ros::Rate out_rate(1);
        std::vector<WayPoint2> vect_target_pos = std::vector<WayPoint2>();


        if(left_mission)
        {
            curr_pos.x -= x_range;
            curr_pos.speed = mul_left * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.y += y_range;
            curr_pos.speed = mul_front * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.x += (2.0f*x_range);
            curr_pos.speed = mul_right * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.y -= y_range;
            curr_pos.speed = mul_back * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));

            // Home position
            _temp_pos.speed = mul_left * _outdoor_speed;
            _temp_pos.x -= 3.50f;
            vect_target_pos.push_back(converter.convert(_temp_pos));
        }
        else
        {
            curr_pos.x += x_range;
            curr_pos.speed = mul_right * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.y += y_range;
            curr_pos.speed = mul_front * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.x -= (2.0f*x_range);
            curr_pos.speed = mul_left * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));
            curr_pos.y -= y_range;
            curr_pos.speed = mul_back * _outdoor_speed;
            vect_target_pos.push_back(converter.convert(curr_pos));

            // Home position
            _temp_pos.speed = mul_right * _outdoor_speed;
            _temp_pos.x += 3.50f;
            vect_target_pos.push_back(converter.convert(_temp_pos));
        }

        _temp_pos.speed = _outdoor_speed;
        _temp_pos.z = 2.50f;
        vect_target_pos.push_back(converter.convert(_temp_pos));
        

        if(vect_target_pos.size() <= 0)
            return;
        
        WayPoint _alt_high = copter->get_current_position();
        _alt_high.z = _outdoor_alt;
        _alt_high.yaw = 360.0f - _start_pos.yaw;
        copter->Go(_alt_high);
        logger->write_show(LogLevel::INFO, "Waiting to reached altitude %.1f m.", _outdoor_alt);
        while(ros::ok() && !copter->check_alt(_outdoor_alt, 2.0f))
        {
            ros::spinOnce();
            out_rate.sleep();
        }

        // Get first target position
        WayPoint2 temp_pos2 = vect_target_pos.front();
        WayPoint temp_pos1 = {temp_pos2.x, temp_pos2.y, temp_pos2.z, temp_pos2.yaw};
        vect_target_pos.erase(vect_target_pos.begin());
        copter->Go(temp_pos1);
        copter->set_speed(temp_pos2.speed);

        // Check if target position vector is not null or zero
        int cntr = 0;
        bool _timeout_en = false;
        int _timeout_count = 10;

        while(ros::ok())
        {
            if (copter->is_reached(temp_pos1, 3.5f))
            {
                // Servo rotated
                cntr++;
                if(cntr < 5)
                {
                    logger->write_show(LogLevel::INFO, "Dropping payload %d", cntr);
                    this->ser.custom_pwm(10, 1800);
                    ros::Duration(0.53f).sleep();
                    this->ser.custom_pwm(10, 1500);
                }
                ros::Duration(1.0f).sleep();

                // Refresh terget position in temp_pos and erase it
                temp_pos2 = vect_target_pos.front();
                vect_target_pos.erase(vect_target_pos.begin());

                // Go to new target position
                temp_pos1 = {temp_pos2.x, temp_pos2.y, temp_pos2.z, temp_pos2.yaw};
                copter->Go(temp_pos1);
                copter->set_speed(temp_pos2.speed);
                if(vect_target_pos.size() <= 0)
                    break;
            }else{
                copter->Go(temp_pos1, false);
            }
            ros::spinOnce();
            out_rate.sleep();
        }

        // Waiting for the last position is reached;
        logger->write_show(LogLevel::INFO, "Waiting to LAND ");
        while(ros::ok() && !copter->check_alt(3.0f, 2.0f))
        {
            ros::spinOnce();
            out_rate.sleep();
        }

        logger->write_show(LogLevel::INFO, "Finished Outdoor Mission");
    }

    Misi2::~Misi2()
    {
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavcon_node");
    EMIRO::Misi2 misi(argc, argv);
    if(misi.is_start())
    {
        misi.Indoor();
        misi.Outdoor();
        misi.Land();
    }
    return 0;
}