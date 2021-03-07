/*
 * Description:     The main class for Bachelor project using sound for navigation
 *
 * Author:			Jaccob Fløe Jeppsen
 * Institution: 	University of Southern Denmark
 *
 * Creation date:	13-02-2021
 */

 #include "json-c/json.h"
#include <math.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <array>
#include <iostream>
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <vector>


 //Defines
#include "includes/defines.h"

//Classes
#include "includes/soundLocalization.h"
#include "includes/motorControl.h"
#include "includes/LIDAR.h"
#include "includes/learning.h"
#include "includes/soundLocalization.h"
#include "includes/navigation.h"
#include "includes/ODAS.h"


//#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
//#include <pigpio.h>
#include "json-c/json.h"

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

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>

#define REFLEX_THRESHOLD	80
#define AVOIDANCE_THRESHOLD	65

#define VELOCITY_OFFSET		12


double activation(double input) {
	return 50 / (1 + exp(-input));
}

void braitenberg(double angle, MotorControl * _motorControl) { //Braitenberg aggression vehicle
	if (angle < 180) { //Object is on RIGHT side
		_motorControl->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		_motorControl->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);
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
		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);
	}
	else { // angle >= 180 //object is on LEFT side
		motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);
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
	************************   INITIALISE CLASSES   ************************
	*****************************************************************************/

	MotorControl motorControl;
	motorControl.changeMotorCommand(STOP); //Stops all motors
	ODAS soundLocalization;
	//Vision vision;

	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;

/*****************************************************************************
************************   ICO LEARNING   ************************************
*****************************************************************************/
//	double angle_current = 270.0;
//	double angle_prev;

/*****************************************************************************
************************   CONTROLLER LOOP   *********************************
*****************************************************************************/


    while(true){

        soundLocalization.updateODAS();

		if (soundLocalization.getEnergy() > ENERGY_THRESHOLD)
			braitenberg(soundLocalization.getAngle(), &motorControl);
		else
			motorControl.changeMotorCommand(STOP);


	} // End of while loop

	motorControl.changeMotorCommand(STOP);
	motorControl.resetMatrixVoiceLEDs();

/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	motorControl.setRightMotorSpeedDirection(0,1);
	motorControl.setLeftMotorSpeedDirection(0,1);


	std::cout << "End of main -------" << std::endl;

	return 0;
}
