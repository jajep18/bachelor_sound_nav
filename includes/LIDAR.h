#pragma once
/*
 * Description:     Vision (camera and detection) class
 *
 * Author:			Jacob Fl�e Jeppesen
 *					University of Southern Denmark
 * Creation date:   18-03-2021
 */
 /*
  *  RPLIDAR
  *  Ultra Simple Data Grabber Demo App
  *
  *  Copyright (c) 2009 - 2014 RoboPeak Team
  *  http://www.robopeak.com
  *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
  *  http://www.slamtec.com
  *
  */
#include "../sdk/include/rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <mutex>
#include <cstring>

#include "defines.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif



using namespace rp::standalone::rplidar;
class LIDAR
{


private:
	//LIDAR variables
	RPlidarDriver* lidar;
	const char* opt_com_path = "/dev/ttyUSB0";
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;
	bool useArgcBaudrate = false;
	bool ctrl_c_pressed;
	RPlidarDriver* drv;
	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	rplidar_response_measurement_node_hq_t nodes[8192];		 //Struct contains angle, distance, quality and a flag
	rplidar_response_measurement_node_hq_t dataNodes[8192];

	//Health information
	rplidar_response_device_health_t healthinfo;

	//MUTEX
	std::mutex LIDARMutex;

	//LIDAR methods
	bool checkRPLIDARHealth(RPlidarDriver* drv);
	void writeScan();



public:
	LIDAR();
	~LIDAR();

	void LIDARScan();

	//Uncorrected readings, using the native angles of the lIDAR:
	rplidar_response_measurement_node_hq_t readScan();			//Checks between 90 and 270		= 180 degrees
	rplidar_response_measurement_node_hq_t readScanReflex();	//Checks between 45 and 315		= 90 degrees
	rplidar_response_measurement_node_hq_t readScanObjCheck();	//Checks between 355 and 5		= 10 degrees

    void ctrlc();

    double getCorrectedAngle(rplidar_response_measurement_node_hq_t closestNode); //Corrects the angles from the LIDAR


};
