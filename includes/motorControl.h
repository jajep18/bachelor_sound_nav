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

// GPIO via Matrix Voice, placed on top of robot
#define  TB6612_RIGHT_MOTOR_PWMA        14 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         8  // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        12 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        10 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         6  // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         4  // (Pink)

// Matrix Voice LEDs
#define MATRIX_LED_R_1          0
#define MATRIX_LED_R_2          1
#define MATRIX_LED_R_3          2
#define MATRIX_LED_R_4          3
#define MATRIX_LED_R_5          4
#define MATRIX_LED_R_6          5
#define MATRIX_LED_R_7          6
#define MATRIX_LED_R_8          7
#define MATRIX_LED_R_9          8

#define MATRIX_LED_L_1          9
#define MATRIX_LED_L_2          10
#define MATRIX_LED_L_3          11
#define MATRIX_LED_L_4          12
#define MATRIX_LED_L_5          13
#define MATRIX_LED_L_6          14
#define MATRIX_LED_L_7          15
#define MATRIX_LED_L_8          16
#define MATRIX_LED_L_9          17

// Motor commands
#define	STOP			0
#define FORWARD			1
#define REVERSE		   -1
#define LEFTTURN		2
#define RIGHTTURN		3
#define	FORWARDSPEED	25
#define	REVERSESPEED	20

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

	void initGPIOPins();									//Initializes General Purpose IO Pins

	void setLeftMotorSpeedDirection(int speed, int dir);	//Controls speed and direction of left motor
	void setRightMotorSpeedDirection(int speed, int dir);	//Controls speed and direction of left motor
	void changeMotorCommand(int command, int speedL = 0, int speedR = 0);				    //Changes motor speed and direction from commands: STOP, FORWARD, REVERSE, LEFTTURN, RIGHTTURN
	void steerToAngle(int angle, int speedL, int speedR);

	void startupShowLEDRainbow();							//Shows rainbow of colors to test LEDs
	void setMatrixVoiceLED(int ledn, int r, int g, int b);	//Changes LED 'n' to color from RGB value
	void resetMatrixVoiceLEDs();							//Turns off LEDS on MatroixVoice
};

