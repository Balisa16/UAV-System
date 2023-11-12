#ifndef SERVO_HEADER
#define SERVO_HEADER

#include <copter.hpp>
#include <enum.hpp>
// #include <Logger.hpp>
#include <vector>

namespace EMIRO{
	
	class Servo
	{
	private:
		mavros_msgs::CommandLong srv;
		std::vector<NServo> servo_list;
		std::vector<DSServo> servo_ds_list;
		std::shared_ptr<EMIRO::Copter> copter;
		std::shared_ptr<EMIRO::Logger> logger;
	public:
		Servo();
		void init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger);
		bool custom_pwm(int aux_idx, int ms);
		bool servo_normal(int aux_idx, Servo_Condition cond);
		bool servo_ds(int aux_idx, DSServo_Condition cond);
		~Servo();
	};
}
#endif // SERVO_HEADER

