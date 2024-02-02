#pragma once

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace EMIRO
{
    class TFMini
    {
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

    private:
        ros::Subscriber tf_sub;
        std_msgs::Int32 tf_data;
        void tf_data_sub(const std_msgs::Int32::ConstPtr &msg);
    };
}