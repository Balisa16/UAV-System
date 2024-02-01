#include <tfmini.hpp>

namespace EMIRO
{
    TFMini::TFMini()
    {
    }

    void TFMini::init(ros::NodeHandle *nh, std::string subs_topic)
    {
        tf_sub = (*nh).subscribe<std_msgs::Int32>(subs_topic, 10,
                                                  [this](const std_msgs::Int32::ConstPtr &msg)
                                                  {
                                                      this->tf_data_sub(msg);
                                                  });
    }

    void TFMini::tf_data_sub(const std_msgs::Int32::ConstPtr &msg)
    {
        tf_data = *msg;
    }

    float TFMini::get_data()
    {
        return tf_data.data / 100.0f;
    }

    TFMini::~TFMini()
    {
    }

}