/*
 * Description:     The main class for Bachelor project using sound for navigation
 *
 * Author:			Jaccob Fløe Jeppsen
 * Institution: 	University of Southern Denmark
 *
 * Creation date:	13-02-2021
 */

 //Defines
#include "includes/defines.h"

//Classes
#include "includes/soundLocalization.h"
#include "includes/motorControl.h"
#include "includes/LIDAR.h"
#include "includes/learning.h"
#include "includes/soundLocalization.h"
#include "includes/navigation.h"
//#include "includes/ODAS.h"


#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
//#include <pigpio.h>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

//MatrixHal
#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>


#define REFLEX_THRESHOLD	80
#define AVOIDANCE_THRESHOLD	65

#define VELOCITY_OFFSET		12




double test_angle = 270;
double ICO_weight = 1;
 //odas.updateODAS(); &//dummy: getOdasAngle(); returns double angle
double getODASAngle() {
	double res = test_angle;
	if (test_angle > 180) {
		test_angle -= 10;
	}

	return res;
}

double activation(double input) {
	return 50 / (1 + exp(-input));
}

void braitenberg(double angle, MotorControl * _motorControl) { //Braitenberg aggression vehicle
	if (angle < 180) { //Object is on RIGHT side
		_motorControl->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 255, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		_motorControl->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 255, 0);
	}

	// Update sensor signals
	double angleL = (((360 - angle) - 180) / 180); // Normalize
	double angleR = (angle-180) / 180; // Normalize

//	motorControl->setRightMotorSpeedDirection(activation(angleR) + VELOCITY_OFFSET, 1);
//	motorControl->setLeftMotorSpeedDirection(activation(angleL) + VELOCITY_OFFSET, 1);

    int motorSpeedL = activation(angleL) + VELOCITY_OFFSET;
    int motorSpeedR = activation(angleR) + VELOCITY_OFFSET;

    std::cout << "Motorspeed left: " << motorSpeedL << ". Motorspeed right: " << motorSpeedR << std::endl;

    _motorControl->changeMotorCommand(FORWARD,motorSpeedL,motorSpeedR);
}


void navigationICO(double angle, MotorControl * motor_control, double w_A) {
	if (angle < 180) { //Object is on RIGHT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, 255, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, 255, 0);
	}

	// Update sensor signals
	double angleL = (((360 - angle) - 180) / 180); // Normalize
	double angleR = (angle - 180) / 180; // Normalize

	motor_control->setRightMotorSpeedDirection(activation(angleR)*w_A + VELOCITY_OFFSET, 1);
	motor_control->setLeftMotorSpeedDirection(activation(angleL)*w_A + VELOCITY_OFFSET, 1);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activation(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activation(angleR) + VELOCITY_OFFSET) << std::endl;
}



int main(int argc, char** argv)
{
	/*****************************************************************************
	************************   INITIALISE MOTOR CONTROL   ************************
	*****************************************************************************/

	MotorControl motorControl;
	//ODAS odas;
	//Vision vision;

	motorControl.changeMotorCommand(STOP, STOP, STOP);
/*********************************   DONE   *********************************/


	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;

/*****************************************************************************
************************   ICO LEARNING   ************************************
*****************************************************************************/
	double angle_current = 270.0;
	double angle_prev;


/*****************************************************************************
************************   CONTROLLER LOOP   *********************************
*****************************************************************************/
    //odas.updateODAS();
    for(int i = 0; i < 15;i++)	{
		//odas.updateODAS();
		//vision.updateCamera();

		angle_prev = angle_current;
		angle_current = getODASAngle();

		braitenberg(angle_current,&motorControl);
		usleep(500000);


        w_A = (abs(angle_current - 180) - abs(angle_prev - 180))/180 * S_L + (1 - S_L) * w_A;

	} // End of while loop

	motorControl.changeMotorCommand(STOP, STOP, STOP);
	motorControl.resetMatrixVoiceLEDs();
/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	//motor_control.setRightMotorSpeedDirection(0,1);
	//motor_control.setLeftMotorSpeedDirection(0,1);

	//Test flag
	std::cout << "End of main -------" << std::endl;




	return 0;
}


//int main(int argc, char** argv)
//{
//	/************************   INITIALISE MOTOR CONTROL   ************************/
//
//	MotorControl motorControl;
//
//	/*********************************   DONE   *********************************/
//
//	// Wait 3 seconds for camera image to stabilise
//	std::cout << "Waiting 3 seconds for setup...\n";
//	usleep(3000000);
//	std::cout << "Done. \n";
//
//
//	/************************   CONTROLLER LOOP   *********************************/
//	int run = 0;
//	while (run < 2)
//	{
//		motorControl.changeMotorCommand(LEFTTURN);
//		usleep(3000000);
//		motorControl.changeMotorCommand(RIGHTTURN);
//		usleep(3000000);
//
//		run++;
//		std::cout << run << "\n";
//
//	} // End of while loop
///*********************************   END OF CONTROLLER LOOP   *********************************/
//
//	// Stop all motors
//	motorControl.setRightMotorSpeedDirection(0, 1);
//	motorControl.setLeftMotorSpeedDirection(0, 1);
//
//
//
//
//	return 0;
//}
