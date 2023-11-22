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
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
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
#include <type_traits>
#include <param.hpp>

#ifdef IS_JETSON_PLATFORM
  #include <pin.hpp>
#endif

#include <Logger.hpp>
#include <enum.hpp>

namespace EMIRO {
  extern const std::string COPTER_DIR;

#ifndef POSITION_STRUCT
#define POSITION_STRUCT
	typedef struct{
		float x, y, z;
	}Position;
#endif

#ifndef QUATERNION_STRUCT
#define QUATERNION_STRUCT
	typedef struct{
		float w, x, y, z;
	}Quaternion;
#endif

  class Copter {
  private:
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
    std::shared_ptr<EMIRO::Logger> logger;
    EMIRO::Logger traj_logger;
    Param copter_param;

    // Initialize frame
    WayPoint local_frame;
    bool is_init_pubs_subs, is_init_frame;

    // System variable
    float local_offset_g;
    int pin_1 = 7;
    int pin_2 = 16;
    int pin_3 = 18;
    float copter_speed = 1.0f;

    // Remote
    uint16_t rc6_pwm = 1000;
    uint16_t rc7_pwm = 1000;

    // Function
    Quaternion to_quaternion(float roll_rate, float pitch_rate, float yaw_rate);
    geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);
    void pose_cb_local(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pose_cb_global(const geographic_msgs::GeoPoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    geometry_msgs::Point get_hexa_point();
    void go_to(geometry_msgs::Pose pose);
    void goto_xyz_rpy(float x, float y, float z, float roll, float pitch, float yaw);
    void viso_align();
    int land();


  public:
    ros::ServiceClient command_client;
    void print_wp(std::string header, WayPoint& wp);

    Copter();

    /**
     * @brief     This function initializes all service clients, publishers, and
     * subscribers.
     *
     * @param nh  A reference to the Node Handle that manages services,
     * publishers, and clients
     */
    void init(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<EMIRO::Logger> logger);

    /**
     * @brief           Waiting for FCU to connect
     * 
     * @param timeout_s Timeout for connect (second)
     * @return false:   Failed to connect FCU.
     * @return true:    FCU Connected successfully
     */
    bool FCUconnect(float timeout_s = 1.0f);

    /**
     * @brief           Waiting FCU turn into GUIDED Mode.
     *
     * @param timeout_s Timeout for connect (second)
     * @return true     FCU in GUIDED mode
     * @return false    FCU not in GUIDED mode
     */
    bool FCUstart(float timeout_s = 1.0f);

    void init_frame(float timeout_s = 1.0f);

    /**
     * @brief Set drone mode.
     *
     * @param mode    Copter mode (Capitatize)
     * @see https://ardupilot.org/copter/docs/flight-modes.html
     * @return int
     */
    int set_mode(CopterMode mode);
    
    void set_vel(float vx, float vy, float vz, float avx, float avy, float avz);

    /**
     * @brief       This function arming drone throttle.
     *
     * @return int  Service feedback
     */
    bool Arming();

    /**
     * @brief Arming drone and takeoff in X = 0, Y = 0
     *
     * @param takeoff_alt   Target altitude (m)
     * @return int          Service feedback
     */
    int takeoff(float takeoff_alt);

    int just_takeoff(float takeoff_alt, float _yaw);

    /**
     * @brief       Arming drone and takeoff in X = wp.x, Y = wp.y
     * @note        Used when we want to takeoff not in home position / EKF origin
     * position
     *
     * @param wp    Current Waypoint
     * @return int
     */
    int takeoff2(WayPoint wp);

    /**
     * @brief     Give command to copter to go to desire Waypoint
     *
     * @param wp  Target point
     */
    void Go(WayPoint &wp, bool show = false, std::string header = "Go to");

    float get_alt();
    
    /**
     * @brief     Landing drones in current in X = wp.x  and Y = wp.y position
     *
     * @param wp
     */
    void Go_Land(WayPoint wp, float tolerance = 0.15f);

    /**
     * @brief Just Send Land Command
     *
     */
    void Land();

    /**
     * @brief             Set drone speed besed Ground Speed of drones.
     * DO_CHANGE_SPEED
     *
     * @param speed_mps   Desire speed (m/s)
     *
     * @return int        Service feedback
     */
    int set_speed(float speed_mps);

    /**
     * @brief             Set the EKF Source. Can be GPS or Non-GPS (T265 Camera)
     * 
     * @param source      EKF Source Type
     */
    void set_ekf_source(EKF_Source source);

    /**
     * @brief       Set the EKF Origin for the desired GPS global.
     *
     * @param lat   Desire latitude
     * @param lnt   Desire longitude
     * @param alt   Desire altitude (m)
     *
     * @note This parameter just can be once setting
     */
    void set_ekf_origin(float lat, float lnt, float alt);

    /**
     * @brief Set the home position.
     * @note This parameter can be set more than once. But be carefully when drone
     * land and takeoff again, home automatically set into takeoff position
     *
     * @param lat   Desire latitude
     * @param lnt   Desire longitude
     * @param alt   Desire altitude
     *
     * @return int  Service feedback
     */
    int set_home(float lat, float lnt, float alt);

    /**
     * @brief             Check if copter in range of tolerance waypoint target.
     *
     * @param dest        Waypoint target
     * @param tolerance   Tolerance range (m)
     * @return true       Copter successfully reached the waypoint
     * @return false      Copter isn't reached the waypoint
     */
    bool is_reached(WayPoint dest, float tolerance);

    bool check_alt(float dist_alt, float tolerance);

    WayPoint calc_transition(WayPoint start_point, WayPoint stop_point, float copter_deg, float copter_alt = 0.8f);

    // Setter
    void set_rc(int channel, int pwm);

    // Getter

    /**
     * @brief         Get current drone's yaw
     *
     * @return float  Degree of drone's yaw
     */
    float get_yaw(bool use360 = false);

    // WayPoint and WayPointG
    template <typename T>
    void get_position(T& pose_ref);

    Mode get_current_mission();

    ~Copter();
  };

}

#endif // COPTER_HEADER
