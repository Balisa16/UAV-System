#include <UART.hpp>
#include <copter.hpp>
#include <memory>

int main()
{
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	std::shared_ptr<ros::NodeHandle> logger;
	logger = std::make_shared<EMIRO::Logger>();
	logger->init("Copter", FileType::CSV);
	logger->start(true);

	EMIRO::Copter copter;
	copter.init(nh, logger);

	EMIRO::UART master_uart;
	master_uart.init("/dev/ttyTHS1", B9600);

	int counter = 4;
	EMIRO::Position pos;
	EMIRO::Quaternion quat;
	
	while(true)
	{
		// Read current position

		// Send current position
		master_uart.write_pose(&pos, &quat);
		counter--;
	}
	return 0;
}
