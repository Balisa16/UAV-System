#ifndef RANGE_FINDER_HPP
#define RANGE_FINDER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace EMIRO{
    class RangeFinder
    {
    private:
        ros::Subscriber rangefinder_sub;
        void rangefinder_cb(const sensor_msgs::Range::ConstPtr& msg);
        float range;
    public:
        RangeFinder();
        void init(ros::NodeHandle *nh);
        float get_range();
        ~RangeFinder();
    };

    RangeFinder::RangeFinder()
    {
        
    }

    inline void RangeFinder::init(ros::NodeHandle *nh)
    {
        rangefinder_sub = (*nh).subscribe<sensor_msgs::Range>("/mavros/rangefinder/rangefinder", 2, 
        [this](const sensor_msgs::Range::ConstPtr& msg) {
            this->rangefinder_cb(msg);
        });
    }

    inline float RangeFinder::get_range()
    {
        return range;
    }

    inline void RangeFinder::rangefinder_cb(const sensor_msgs::Range::ConstPtr& msg)
    {
        range = msg->range;
    }


    RangeFinder::~RangeFinder()
    {
    }
}

#endif