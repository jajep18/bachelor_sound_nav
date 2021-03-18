#pragma once
/*
 * Description:     Vision (camera and detection) class
 *
 * Author:			Jacob Fløe Jeppesen
 *					University of Southern Denmark
 * Creation date:   18-03-2021
 */
#include <rplidar.h>
#include <iostream>
#include "defines.h"

class LIDAR
{

private:
	RPlidarDriver* lidar;

public:
	LIDAR();
	~LIDAR();

};

