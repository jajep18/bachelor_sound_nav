#pragma once
/*
 * Description:     ODAS class - Implementation of ODAS and sound localization for MATRIX VOICE.
 *							   - Listens to odas live (has to be run from same terminal before this class is instantiated)
 *
 * Author:			Erik Lindby & Jacob Fløe Jeppesen
 *					Derivative of MATRIX Labs ODAS
 *					University of Southern Denmark
 *
 * Creation date:   22-02-2021
 */
#include "json-c/json.h"
#include <math.h>
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/matrixio_bus.h>
#include <matrix_hal/gpio_control.h>
#include "matrix_hal/microphone_array.h"
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <iostream>
#include <thread>
#include <mutex>

#include "defines.h"

class ODAS
{
private:

	double x, y, z, E;
    int angle = -1, currentMax = -1, prevMax = -1, currentMaxIndex;
    int prevAngle = -1, energy = -1;
	std::vector<int> angleVec;
	int testFlag = 0;
	int energyArray[ENERGY_COUNT] = { 0 };
	const double led_angles_mvoice[18] = { 170, 150, 130, 110, 90,  70,
                                           50,  30,  10,  350, 330, 310,
                                           290, 270, 250, 230, 210, 190 }; //LED angles for MATRIX Voice
	int ledCount;
	matrix_hal::MatrixIOBus* bus;				// Create MatrixIOBus object for hardware communication
	matrix_hal::EverloopImage* image1d;			// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop* everloop;				// Create Everloop object

	//Connection variables
	char verbose = 0x00;
	int server_id;
	struct sockaddr_in server_address;
	int connection_id;
	char* message;
	int messageSize;
	int c;
	unsigned int portNumber = 9001;
	const unsigned int nBytes = 10240;

	void increase_pots();
	void decrease_pots();
	void json_parse(json_object* jobj);
	void json_parse_array(json_object* jobj, char* key);


	std::mutex angleEnergyMutex;
public:

	ODAS(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* image1d_);
	~ODAS();

	void updateODAS();

	std::vector<int> getEnergyArray();
	double getSoundAngle();

	void updateSoundInformation();

	int getAngle();
    int getEnergy();

};

