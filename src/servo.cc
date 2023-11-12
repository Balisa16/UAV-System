#include <servo.hpp>

namespace EMIRO{
	Servo::Servo()
	{
		std::cout << "Created servo\n";
	}
	
	void Servo::init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger)
	{
		this->logger = logger;
		this->copter = copter;
		servo_list.push_back({5, 1000, 2000});
		servo_list.push_back({6, 1000, 2000});
		servo_list.push_back({3, 1000, 2000});
		servo_list.push_back({4, 1000, 2000});
		servo_ds_list.push_back({1, 1000, 1500, 2000});
		servo_ds_list.push_back({2, 1000, 1500, 2000});
		logger->write_show(LogLevel::INFO, "Servo registered");
	}

	bool Servo::custom_pwm(int aux_idx, int ms)
	{
		srv.request.command = 183; 
		srv.request.param1 = aux_idx;
		srv.request.param2 = ms;
		return copter->command_client.call(srv);
	}

	bool Servo::servo_normal(int aux_idx, Servo_Condition cond)
	{
		if(aux_idx > 6 || aux_idx < 1)
		{
			logger->write_show(LogLevel::ERROR, "Servo %d invalid. Please use index 1 - 6", aux_idx);
			return false;
		}
		
		for (int i = 0; i < servo_list.size(); i++)
			if(servo_list[i].Aux_idx == aux_idx)
			{
				srv.request.command = 183;
				srv.request.param1 = aux_idx + 8;
				srv.request.param2 = servo_list[i].Open;
				if (cond == Servo_Condition::Close)
					srv.request.param2 = servo_list[i].Close;
				return copter->command_client.call(srv);
			}
		return false;
	}

	bool Servo::servo_ds(int aux_idx, DSServo_Condition cond)
	{
		if(aux_idx > 6 || aux_idx < 1)
		{
			logger->write_show(LogLevel::ERROR, "Servo %d invalid. Please use index 1 - 6", aux_idx);
			return false;
		}
		
		for (int i = 0; i < servo_list.size(); i++)
			if(servo_ds_list[i].Aux_idx == aux_idx)
			{
				srv.request.command = 183;
				srv.request.param1 = aux_idx + 8;

				srv.request.param2 = servo_ds_list[i].Standby;
				if (cond == DSServo_Condition::Left)
					srv.request.param2 = servo_ds_list[i].Left;
				else if (cond == DSServo_Condition::Right)
					srv.request.param2 = servo_ds_list[i].Right;
				return copter->command_client.call(srv);
			}
		return false;
	}

	Servo::~Servo()
	{
	}

}

