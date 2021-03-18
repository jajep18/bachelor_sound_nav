#include "../includes/LIDAR.h"

LIDAR::LIDAR()
{
	lidar = RPlidarDriver::CreateDriver();

	u_result res = lidar->connect("/dev/ttyUSB0", 115200);

	if (IS_OK(res))
	{
		std::cout << "Lidar connected!\n";
		lidar->startMotor();
		std::cout << "Lidar motor started!\n";

		for (size_t i = 0; i < 10; i++)
		{
			std::cout << "Waiting... \n";
		}
		
	}
	else
	{
		fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
	}

	lidar->stopMotor();
	std::cout << "Lidar motor stopped!\n";
	lidar->disconnect();
	std::cout << "Lidar disconnected!\n";
}

LIDAR::~LIDAR()
{
	RPlidarDriver::DisposeDriver(lidar);
}
