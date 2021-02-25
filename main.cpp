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



int main(int argc, char** argv)
{
	/*****************************************************************************
	************************   INITIALISE MOTOR CONTROL   ************************
	*****************************************************************************/

	//MotorControl motor_control;
	ODAS odas;
	//Vision vision;

/*********************************   DONE   *********************************/


	// Wait 3 seconds for camera image to stabilise
	//cout << "Waiting for camera to stabilise...";
	//usleep(3000000);
	//cout << "done." << endl;


/*****************************************************************************
************************   CONTROLLER LOOP   *********************************
*****************************************************************************/
	odas.updateODAS();
	//while(true)
	//{
		//odas.updateODAS();
		//vision.updateCamera();



	//} // End of while loop
/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	//motor_control.setRightMotorSpeedDirection(0,1);
	//motor_control.setLeftMotorSpeedDirection(0,1);

	//Test flag
	cout << "End of main -------" << endl;




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
