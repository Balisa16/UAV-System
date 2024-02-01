#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace EMIRO
{
    class RangeFinder
    {
    private:
        ros::Subscriber rangefinder_sub;
        void rangefinder_cb(const sensor_msgs::Range::ConstPtr &msg);
        float range;

    public:
        RangeFinder();
        void init(ros::NodeHandle *nh);
        float get_range();
        ~RangeFinder();
    };
}