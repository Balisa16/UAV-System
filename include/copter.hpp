#ifndef COPTER_HEADER
#define COPTER_HEADER

// ROS Library
#include <ros/duration.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

// Mavros Message
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandLongRequest.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// GEometry Message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Additional Library
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cinttypes>
#include <cstdio>
#include <param.hpp>
#include <type_traits>

#ifdef IS_JETSON_PLATFORM
#include <jetson/pin.hpp>
#endif

#include <Logger.hpp>
#include <control.hpp>
#include <enum.hpp>
#include <gps.hpp>

namespace EMIRO
{
    extern const std::string COPTER_DIR;

    class Copter
    {
    public:
        static ros::NodeHandle *get_nh(int argc = 0, char **argv = nullptr)
        {
            static ros::NodeHandle nh;
            return &nh;
        }

        static Copter &get(int argc = 0, char **argv = nullptr)
        {
            static Copter copter_instance;
            return copter_instance;
        }

        static Logger &get_logger()
        {
            static Logger main_logger;
            return main_logger;
        }

        static bool init(int argc, char **argv, std::string copter_name = "copter", std::string log_name = "copter", FileType log_type = FileType::CSV);

        /**
         * @brief Init home position
         *
         * @param timeout_s timeout of init (second)
         */
        static void init_frame(float timeout_s = 1.0f);

        /**
         * @brief           Waiting for FCU to connect
         *
         * @param timeout_s Timeout for connect (second)
         * @return false:   Failed to connect FCU.
         * @return true:    FCU Connected successfully
         */
        static bool FCUconnect(float timeout_s = 1.0f);

        /**
         * @brief           Waiting FCU turn into GUIDED Mode.
         *
         * @param timeout_s Timeout for connect (second)
         * @return true     FCU in GUIDED mode
         * @return false    FCU not in GUIDED mode
         */
        static bool FCUstart(float timeout_s = 1.0f);

        /**
         * @brief Set drone mode.
         *
         * @param mode    Copter mode (Capitatize)
         * @see https://ardupilot.org/copter/docs/flight-modes.html
         * @return int
         */
        static int set_mode(CopterMode mode);

        /**
         * @brief Set the linear velocity and angular velocity
         *
         * @param vx  linear velocity of x (m/s)
         * @param vy  linear velocity of y (m/s)
         * @param vz  linear velocity of z (m/s)
         * @param avx angular velocity of x (rad/s)
         * @param avy angular velocity of y (rad/s)
         * @param avz angular velocity of z (rad/s)
         */
        static void set_vel(const float &vx, const float &vy, const float &vz,
                            const float &avx, const float &avy, const float &avz);

        /**
         * @brief Set the linear velocity and angular velocity
         *
         * @param cmd_vel  copter linear and angular velocity.
         * @see void set_vel(float &vx, float &vy, float &vz, float &avx, float
         * &avy, float &avz);
         */
        static void set_vel(geometry_msgs::Twist &cmd_vel);

        /**
         * @brief       This function arming drone throttle.
         *
         * @return int  Service feedback
         */
        static bool Arming();

        /**
         * @brief Arming drone and takeoff in X = 0, Y = 0
         *
         * @param takeoff_alt   Target altitude (m)
         * @return int          Service feedback
         */
        static int takeoff(float takeoff_alt, float tolerance = 0.2f);

        /**
         * @brief     Give command to copter to go to desire Waypoint
         *
         * @param wp  Target point
         */
        static void Go(WayPoint &wp, bool show = false,
                       std::string header = "Go to");

        static float get_alt();

        /**
         * @brief     Landing drones in current in X = wp.x  and Y = wp.y
         position
         *
         * @param wp
         */
        static void Go_Land(WayPoint wp, float tolerance = 0.15f);

        /**
         * @brief Just Send Land Command
         *
         */
        static void Land();

        /**
         * @brief             Set drone speed besed Ground Speed of drones.
         * DO_CHANGE_SPEED
         *
         * @param speed_mps   Desire speed (m/s)
         *
         * @return int        Service feedback
         */
        static int set_speed(float speed_mps);

        /**
         * @brief             Set the EKF Source. Can be GPS or Non-GPS (T265
         * Camera)
         *
         * @param source      EKF Source Type
         */
        static void set_ekf_source(EKF_Source source);

        /**
         * @brief       Set the EKF Origin for the desired GPS global.
         *
         * @param lat   Desire latitude
         * @param lnt   Desire longitude
         * @param alt   Desire altitude (m)
         *
         * @note This parameter just can be once setting
         */
        static void set_ekf_origin(float lat, float lnt, float alt);

        /**
         * @brief Set the home position.
         * @note This parameter can be set more than once. But be carefully when
         * drone land and takeoff again, home automatically set into takeoff
         * position
         *
         * @param lat   Desire latitude
         * @param lnt   Desire longitude
         * @param alt   Desire altitude
         *
         * @return int  Service feedback
         */
        static int set_home(float lat, float lnt, float alt);

        /**
         * @brief             Check if copter in range of tolerance waypoint
         target.
         *
         * @param dest        Waypoint target
         * @param tolerance   Tolerance range (m)
         * @return true       Copter successfully reached the waypoint
         * @return false      Copter isn't reached the waypoint
         */
        static bool is_reached(WayPoint dest, float tolerance);

        static bool check_alt(float dist_alt, float tolerance);

        static WayPoint calc_transition(WayPoint start_point, WayPoint stop_point,
                                        float copter_deg, float copter_alt = 0.8f);

        // Setter
        static void set_rc(int channel, int pwm);

        // Getter
        static void get_pose(Position *pos, Quaternion *quat);

        /**
         * @brief         Get current drone's yaw
         *
         * @return float  Degree of drone's yaw
         */
        static float get_yaw(bool use360 = false);

        // WayPoint and WayPointG
        static void get_position(WayPoint &pose_ref);

        static Mode get_current_mission();

        static void go_rtl(float alt = -1.f, float tolerance = 0.2f);

    private:
        Copter();

        // Prevent Copying
        Copter(const Copter &) = delete;
        Copter(Copter &&) = delete;
        Copter &operator=(const Copter &) = delete;
        Copter &operator=(Copter &&) = delete;
        virtual ~Copter();

        // Static Implementation
        bool copter_init(std::string logger_name, FileType logger_type);
        void copter_init_frame(float timeout_s);
        bool copter_FCUconnect(float timeout_s = 1.0f) const;
        bool copter_FCUstart(float timeout_s = 1.0f) const;
        int copter_set_mode(CopterMode mode);
        void copter_set_vel(const float &vx, const float &vy, const float &vz, const float &avx, const float &avy, const float &avz);
        void copter_set_vel(geometry_msgs::Twist &cmd_vel);
        int copter_set_speed(float speed_mps);
        void copter_set_ekf_source(EKF_Source source);
        void copter_set_ekf_origin(float lat, float lnt, float alt);
        int copter_set_home(float lat, float lnt, float alt);
        void copter_set_rc(int channel, int pwm);
        bool copter_Arming();
        int copter_takeoff(float takeoff_alt, float tolerance);
        void copter_Go(WayPoint &wp, bool show = false, std::string header = "Go to");
        void copter_Go_Land(WayPoint wp, float tolerance = 0.15f);
        void copter_Land();
        bool copter_is_reached(WayPoint dest, float tolerance) const;
        bool copter_check_alt(float dist_alt, float tolerance) const;
        WayPoint copter_calc_transition(WayPoint start_point, WayPoint stop_point, float copter_deg, float copter_alt = 0.8f) const;
        float copter_get_alt() const;
        void copter_get_pose(Position *pos, Quaternion *quat) const;
        float copter_get_yaw(bool use360 = false) const;
        void copter_get_position(WayPoint &pose_ref) const;
        Mode copter_get_current_mission() const;
        void copter_Go_RTL(float alt, float tolerance);

        // Private Implementation
        Quaternion _to_quaternion(float roll_rate, float pitch_rate, float yaw_rate) const;
        geometry_msgs::Point _enu_2_local(nav_msgs::Odometry current_pose_enu) const;
        void _pose_cb_local(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void _pose_cb_global(const geographic_msgs::GeoPoseStamped::ConstPtr &msg);
        void _state_cb(const mavros_msgs::State::ConstPtr &msg);
        geometry_msgs::Point _get_hexa_point() const;
        void _go_to(geometry_msgs::Pose pose);
        void _goto_xyz_rpy(float x, float y, float z, float roll, float pitch, float yaw);
        void _viso_align() const;
        int _land();

    public:
        ros::ServiceClient command_client;
        CopterStatus status = CopterStatus::None;
        void print_wp(std::string header, WayPoint &wp) const;

    private:
        EMIRO::Logger traj_logger;
        geographic_msgs::GeoPoseStamped pose_data_global;
        geometry_msgs::PoseStamped pose_data_local;
        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::PoseStamped copter_orientation;
        geometry_msgs::Pose pose_data2;
        geometry_msgs::Pose hexa_pose;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point local_offset_pose_g;

        mavros_msgs::State current_state_g;

        ros::Time timestamp;
        ros::Time last_request;

        // Subscriber
        ros::Subscriber cmd_pos_sub_local;
        ros::Subscriber cmd_pos_sub_global;
        ros::Subscriber state_sub;

        // Publisher
        ros::Publisher cmd_pos_pub;
        ros::Publisher gps_pos_pub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher cmd_rc_pub;
        ros::Publisher cmd_rc_override_pub;

        // Service Client
        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client;
        ros::ServiceClient takeoff_client;
        ros::ServiceClient land_client;
        Mode misi_mode;

        // Parameter settings
        Param copter_param;

        // Initialize frame
        WayPoint local_frame, takeoff_wp = {0.0f, 0.0f, 0.0f, 0.0f};
        bool is_init_pubs_subs, is_init_frame;

        // System variable
        float local_offset_g;
        int pin_1 = 7;
        int pin_2 = 16;
        int pin_3 = 18;
        float copter_speed = 1.0f;

        // RC
        uint16_t rc6_pwm = 1000;
        uint16_t rc7_pwm = 1000;
    };

} // namespace EMIRO

#endif // COPTER_HEADER
