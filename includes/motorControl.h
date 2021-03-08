#pragma once
/*
 * Description:     Class for control of motor and MatrixLEDs
 *
 * Author:			Jaccob Fløe Jeppsen
 * Institution: 	University of Southern Denmark
 *
 * Creation date:	13-02-2021
 */

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <unistd.h>

//Classes
#include "defines.h"



class MotorControl {
private:
	int ledCount, action = 1, motorL, motorR;

	matrix_hal::MatrixIOBus* bus;				// Create MatrixIOBus object for hardware communication
	matrix_hal::EverloopImage* everloop_image;	// Create EverloopImage object, with size of ledCount
	matrix_hal::Everloop* everloop;				// Create Everloop object
	matrix_hal::GPIOControl* gpio;				// Create GPIOControl object - General Purpose Input Output

public:
	MotorControl(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* everloop_image_, matrix_hal::GPIOControl* gpio_);
	~MotorControl();

	void initGPIOPins();														//Initializes General Purpose IO Pins

	void setLeftMotorSpeedDirection(int speed, int dir);						//Controls speed and direction of left motor
	void setRightMotorSpeedDirection(int speed, int dir);						//Controls speed and direction of left motor
	void changeMotorCommand(int command, int speedL = 0, int speedR = 0);		//Changes motor speed and direction from commands: STOP, FORWARD, REVERSE, LEFTTURN, RIGHTTURN
	void steerToAngle(int angle, int speedL, int speedR);

	void startupShowLEDRainbow();							//Shows rainbow of colors to test LEDs
	void setMatrixVoiceLED(int ledn, int r, int g, int b);	//Changes LED 'n' to color from RGB value
	void resetMatrixVoiceLEDs();							//Turns off LEDS on MatroixVoice
};

