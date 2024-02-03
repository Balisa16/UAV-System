#ifndef SERVO_HEADER
#define SERVO_HEADER

#include <Logger.hpp>
#include <copter.hpp>
#include <enum.hpp>
#include <vector>

namespace EMIRO {

class Servo {
  public:
    Servo();

    void init(std::shared_ptr<EMIRO::Copter> copter,
              std::shared_ptr<EMIRO::Logger> logger);

    bool custom_pwm(int aux_idx, int ms);

    bool servo_normal(int aux_idx, Servo_Condition cond);

    bool servo_ds(int aux_idx, DSServo_Condition cond);

    ~Servo();

  private:
    mavros_msgs::CommandLong srv;
    std::vector<NServo> servo_list;
    std::vector<DSServo> servo_ds_list;
    std::shared_ptr<EMIRO::Copter> copter;
    std::shared_ptr<EMIRO::Logger> logger;
};
} // namespace EMIRO
#endif // SERVO_HEADER
