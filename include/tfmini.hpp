#ifndef TF_MINI
#define TF_MINI

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace EMIRO{
    class TFMini
    {
    private:
        ros::Subscriber tf_sub;
        std_msgs::Int32 tf_data;
        void tf_data_sub(const std_msgs::Int32::ConstPtr& msg);
    public:
        TFMini();
        /**
         * @brief 
         * 
         * @param nh NodeHandle Object
         * @param subs_topic TFMini topic to substribe
         */
        void init(ros::NodeHandle *nh, std::string subs_topic = "/tfmini");

        /**
         * @brief Get the TFMini Measurement data
         * 
         * @return float Measured distance (in meters)
         */
        float get_data();
        ~TFMini();
    };
    
    TFMini::TFMini()
    {
    }

    inline void TFMini::init(ros::NodeHandle *nh, std::string subs_topic)
    {
        tf_sub = (*nh).subscribe<std_msgs::In2>(subs_topic, 10,
          [this](const std_msgs::Int32::ConstPtr& msg){
            this->tf_data_sub(msg);
          });
    }

    inline void TFMini::tf_data_sub(const std_msgs::Int32::ConstPtr& msg)
    {
        tf_data = *msg;
    }

    inline float TFMini::get_data()
    {
        return tf_data / 100.0f;
    }
    
    TFMini::~TFMini()
    {
    }
    
}

#endif