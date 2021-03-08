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
#include <math.h>

class navigation
{
private:

	double angleL = 0, angleR = 0;
	int motorSpeedL = 0, motorSpeedR = 0, motorCommand = 0;

	MotorControl* motorControl;

	double activationFunction(double input);


public:

    navigation(MotorControl* motorControl_);

	void braitenberg(double angle, std::ofstream & outputBraitenberg);				//Braitenberg aggression vehicle

	void navigationICO(double angle, double w_A);	//ICO learning - work in progress

};

