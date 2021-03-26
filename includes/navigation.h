#pragma once
/*
 * Description:     ODAS class - implementation of ODAS for MATRIX VOICE.
 *							   - Listens to odas live (has to be run from same terminal before this class is instantiated)
 *
 * Author:			Jacob Fløe Jeppesen
 * Institute:		University of Southern Denmark
 * Creation date:   06-03-2021
 */
#include "motorControl.h"
#include "camera.h"

#include <math.h>
#include <fstream>

class navigation
{
private:

	double angleL = 0, angleR = 0;
	double motorSpeedL = 0, motorSpeedR = 0;
	int motorCommand = 1;

	MotorControl* motorControl;



public:

    navigation(MotorControl* motorControl_);

	void braitenberg(double angle, std::ofstream& outputStream, double avoidanceLeft = 0, double avoidanceRight = 0) ;				//Braitenberg aggression vehicle

	void navigationICO(double angle, double w_A);	//ICO learning - work in progress

	void manualInputSteering(Vision * vision_);

	double activationFunction(double input);


};

