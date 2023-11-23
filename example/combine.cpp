#include <UART.hpp>
#include <copter.hpp>
#include <memory>

int main()
{
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	std::shared_ptr<EMIRO::Logger> logger;
	logger = std::make_shared<EMIRO::Logger>();
	logger->init("Copter", EMIRO::FileType::CSV);
	logger->start(true);

	EMIRO::Copter copter;
	copter.init(nh, logger);

	EMIRO::UART master_uart;
	master_uart.init("/dev/ttyTHS1", B9600);

	int cnt = 4;
	int sub_cnt = 3;
	EMIRO::Position pos;
	EMIRO::Quaternion quat;
	
	ros::Rate rate(1);
	while(ros::ok() && cnt)
	{
		if(sub_cnt)
		{
			// Read current position
			
			// Send current position
			master_uart.write_pose(&pos, &quat);
			cnt--;
			sub_cnt = 3;
		}

		ros::spinOnce();
    	rate.sleep();
		sub_cnt--;
	}
	return 0;
}
