#ifndef COPTER_HEADER
#define COPTER_HEADER

// ROS Library
#include <ros/duration.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
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
#include <mavros_msgs/GPSRAW.h>

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
#include <types.hpp>
#include <gps.hpp>
#include <settings.hpp>

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
        static void init_frame(const float timeout_s = 1.0f);

        /**
         * @brief           Waiting for FCU to connect
         *
         * @param timeout_s Timeout for connect (second)
         * @return false:   Failed to connect FCU.
         * @return true:    FCU Connected successfully
         */
        static bool FCUconnect(const float timeout_s = 1.0f);

        /**
         * @brief           Waiting FCU turn into GUIDED Mode.
         *
         * @param timeout_s Timeout for connect (second)
         * @return true     FCU in GUIDED mode
         * @return false    FCU not in GUIDED mode
         */
        static bool FCUstart(const float timeout_s = 1.0f);

        /**
         * @brief Waiting for Arming
         *
         * @param timeout_s Timeout for connect (second)
         * @return true     FCU in GUIDED mode
         * @return false    FCU not in GUIDED mode
         */
        static bool PreArmedCheck(const float timeout_s = 60.0f);

        /**
         * @brief Waiting for HDOP
         *
         * @param hdop_limit Default value is 1.4. Greater than 1.4 indicates poor GPS tolerance
         * @param duration_ms Timeout for connect (millisecond)
         */
        static void waitHDOP(float hdop_limit = 1.4f, const u_int64_t duration_ms = UINT64_MAX);

        /**
         * @brief Set drone mode.
         *
         * @param mode    Copter mode (Capitatize)
         * @see https://ardupilot.org/copter/docs/flight-modes.html
         * @return int
         */
        static int set_mode(const FlightMode mode);

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
        static int takeoff(const float takeoff_alt, const float tolerance = 0.2f);

        /**
         * @brief     Give command to copter to go to desire Waypoint
         *
         * @param wp  Target point
         */
        static void Go(WayPoint &wp, const bool show = false, const std::string header = "Go to");

        /**
         * @brief Get the drone relative altitude
         *
         * @return float
         */
        static float get_alt();

        /**
         * @brief     Landing drones in current in X = wp.x  and Y = wp.y
         position
         *
         * @param wp
         */
        static void Go_Land(WayPoint &wp, const float tolerance = 0.15f);

        /**
         * @brief Just Send Land Command
         *
         */
        static void Land(const float ground_tolerance = 0.2f);

        /**
         * @brief             Set drone speed besed Ground Speed of drones.
         * DO_CHANGE_SPEED
         *
         * @param speed_mps   Desire speed (m/s)
         *
         * @return int        Service feedback
         */
        static int set_speed(const float speed_mps);

        /**
         * @brief             Set the EKF Source. Can be GPS or Non-GPS (T265
         * Camera)
         *
         * @param source      EKF Source Type
         */
        static void set_ekf_source(const EKFSource source);

        /**
         * @brief Realign the Visual Odometry (VISO) camera
         *
         */
        static void realign_viso();

        /**
         * @brief       Set the EKF Origin for the desired GPS global.
         *
         * @param lat   Desire latitude
         * @param lnt   Desire longitude
         * @param alt   Desire altitude (m)
         *
         * @note This parameter just can be once setting
         */
        static void set_ekf_origin(const float lat, const float lnt, const float alt);

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
        static int set_home(const float lat, const float lnt, const float alt);

        /**
         * @brief             Check if copter in range of tolerance waypoint
         target.
         *
         * @param dest        Waypoint target
         * @param tolerance   Tolerance range (m)
         * @return true       Copter successfully reached the waypoint
         * @return false      Copter isn't reached the waypoint
         */
        static bool is_reached(const WayPoint dest, const float tolerance);

        /**
         * @brief Check altitude
         *
         * @param dist_alt Desire altitude
         * @param tolerance altitude tolerance (m)
         * @return true Altitude is reached
         * @return false Altitude isn't reached
         */
        static bool check_alt(const float dist_alt, const float tolerance);

        /**
         * @brief Calculate transition between two WayPoint
         *
         * @param start_point Start WayPoint
         * @param stop_point Stop WayPoint
         * @param copter_deg Copter's yaw (degree)
         * @param copter_alt Copter's altitude
         * @return WayPoint
         */
        static WayPoint calc_transition(const WayPoint start_point, const WayPoint stop_point,
                                        const float copter_deg, const float copter_alt = 0.8f);

        /**
         * @brief Set the remote control PWM
         *
         * @param channel desire channel
         * @param pwm pwm value. Usually in range of 1000 - 2000
         */
        static void set_rc(const int channel, const int pwm);

        /**
         * @brief Get the copter pose data
         *
         * @param pos Copter position (x(m), y(m), z(m))
         * @param quat Copter quaternion (w, x, y, z)
         */
        static void get_pose(Position *pos, Quaternion *quat);

        /**
         * @brief         Get current drone's yaw
         *
         * @return float  Degree of drone's yaw
         */
        static float get_yaw();

        /**
         * @brief Get the copter position
         *
         * @param pose_ref Waypoint reference
         */
        static void get_position(WayPoint &pose_ref);

        /**
         * @brief Get the GPS Horizontal Dilution of Precision (HDOP)
         *
         * @return float HDOP value
         */
        static float get_hdop();

        /**
         * @brief Get the GPS Vertical Dilution of Precision (VDOP)
         *
         * @return float VDOP value
         */
        static float get_vdop();

        /**
         * @brief Get the GPS satellites number
         *
         * @return int Number of satellites
         */
        static int get_satellites_num();

        /**
         * @brief Get the gps fix type
         *
         * @return GPS_FIX_TYPE Type of GPS fix status
         */
        static GPS_FIX_TYPE get_gps_fix_type();

        /**
         * @brief Get the takeoff position
         *
         * @return WayPoint Position of takeoff
         */
        static WayPoint get_takeoff_position();

        /**
         * @brief Get the current mission object
         *
         * @return Environment Current mission type
         */
        static Environment get_current_mission();

        /**
         * @brief Return to takeoff position
         *
         * @param alt copter takeoff altitude
         * @param tolerance tolerance of position
         */
        static void go_rtl(const float alt = -1.f, const float tolerance = 0.2f);

        /**
         * @brief Set the copter yaw
         *
         * @param _yaw_mode Yaw mode (Relative, Absolute)
         */
        static void set_yaw(const YawMode _yaw_mode);

    private:
        Copter();

        // Prevent Copying
        Copter(const Copter &) = delete;
        Copter(Copter &&) = delete;
        Copter &operator=(const Copter &) = delete;
        Copter &operator=(Copter &&) = delete;
        virtual ~Copter();

        // Static Implementation
        bool copter_init(const std::string logger_name, const FileType logger_type);
        void copter_init_frame(const float timeout_s);
        bool copter_FCUconnect(const float timeout_s = 1.0f) const;
        bool copter_FCUstart(const float timeout_s = 1.0f) const;
        bool copter_PreArmedCheck(uint64_t &cnt) const;
        void copter_waitHDOP(float hdop_limit, u_int64_t duration_ms) const;
        int copter_set_mode(FlightMode mode);
        void copter_set_vel(const float &vx, const float &vy, const float &vz, const float &avx, const float &avy, const float &avz);
        void copter_set_vel(geometry_msgs::Twist &cmd_vel);
        int copter_set_speed(float speed_mps);
        void copter_set_ekf_source(EKFSource source);
        void copter_set_ekf_origin(float lat, float lnt, float alt);
        int copter_set_home(float lat, float lnt, float alt);
        void copter_set_rc(int channel, int pwm);
        bool copter_Arming();
        int copter_takeoff(float takeoff_alt, float tolerance);
        void copter_Go(WayPoint &wp, bool show = false, std::string header = "Go to");
        void copter_Go_Land(WayPoint wp, float tolerance = 0.15f);
        void copter_Land(float tolerance);
        bool copter_is_reached(WayPoint dest, float tolerance) const;
        bool copter_check_alt(float dist_alt, float tolerance) const;
        WayPoint copter_calc_transition(WayPoint start_point, WayPoint stop_point, float copter_deg, float copter_alt = 0.8f) const;
        float copter_get_alt() const;
        void copter_get_pose(Position *pos, Quaternion *quat) const;
        float copter_get_yaw() const;
        void copter_get_position(WayPoint &pose_ref) const;
        float copter_get_hdop() const;
        float copter_get_vdop() const;
        int copter_get_satellites_num() const;
        GPS_FIX_TYPE copter_get_gps_fix_type() const;
        Environment copter_get_current_mission() const;
        void copter_Go_RTL(float alt, float tolerance);
        void copter_realign_viso() const;

        // Private Implementation
        Quaternion _to_quaternion(const float roll_rate, const float pitch_rate, const float yaw_rate) const;
        geometry_msgs::Point _enu_2_local(const nav_msgs::Odometry current_pose_enu) const;
        geometry_msgs::Point _get_hexa_point() const;
        void _go_to(const geometry_msgs::Pose pose);
        void _goto_xyz_rpy(const float x, const float y, const float z, const float roll, const float pitch, const float yaw);
        bool _land(const float tolerance);

    public:
        ros::ServiceClient command_client;
        Status status = Status::None;
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
        mavros_msgs::GPSRAW gps_raw;

        ros::Time timestamp;
        ros::Time last_request;

        // Subscriber
        ros::Subscriber cmd_pos_sub_local;
        ros::Subscriber cmd_pos_sub_global;
        ros::Subscriber state_sub;
        ros::Subscriber gps_raw_sub;

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
        Environment misi_mode;

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

        Eigen::Quaternionf start_quat;
        YawMode yaw_mode = YawMode::ABSOLUTE;
    };

} // namespace EMIRO

#endif // COPTER_HEADER
