#include <rangefinder.hpp>

namespace EMIRO
{
    RangeFinder::RangeFinder()
    {
    }

    void RangeFinder::init(ros::NodeHandle *nh)
    {
        rangefinder_sub = (*nh).subscribe<sensor_msgs::Range>("/mavros/rangefinder/rangefinder", 2,
                                                              [this](const sensor_msgs::Range::ConstPtr &msg)
                                                              {
                                                                  this->rangefinder_cb(msg);
                                                              });
    }

    float RangeFinder::get_range()
    {
        return range;
    }

    void RangeFinder::rangefinder_cb(const sensor_msgs::Range::ConstPtr &msg)
    {
        range = msg->range;
    }

    RangeFinder::~RangeFinder()
    {
    }
}