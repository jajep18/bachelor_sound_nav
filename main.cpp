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
#include "includes/navigation.h"

#include <ctime>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
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



int main(int argc, char** argv)
{
	/*****************************************************************************
	************************   INITIALISE MATRIXIO  ************************
	*****************************************************************************/
	matrix_hal::MatrixIOBus bus;									// Create MatrixIOBus object for hardware communication
	if (!bus.Init())												// Set gpio to use MatrixIOBus bus
		throw("Bus Init failed");
	matrix_hal::EverloopImage everloop_image(bus.MatrixLeds());		// Create EverloopImage object "image1d", with size of ledCount
	matrix_hal::Everloop everloop;									// Create Everloop object
	everloop.Setup(&bus);											// Set everloop to use MatrixIOBus bus
	matrix_hal::GPIOControl gpio;									// Create GPIOControl object - General Purpose Input Output
	gpio.Setup(&bus);
	/*****************************************************************************
	************************   INITIALISE CLASSES  ************************
	*****************************************************************************/
	MotorControl motorControl = MotorControl(&bus, &everloop, &everloop_image, &gpio);
	ODAS soundLocalization = ODAS(&bus, &everloop, &everloop_image);
	navigation navigationObj = navigation(&motorControl);


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

	/*****************************************************************************
	************************   Output stream    **********************************
	*****************************************************************************/

	std::ofstream outputStream;
	outputStream.open("./data/braitenbergMotorCommandsDummy.csv", std::ofstream::out | std::ofstream::trunc);
	//outputStream << "Left angle" << "," << "Activation output left" << "," << "Right angle" << "," << "Activation output right" << std::endl;

	motorControl.setMatrixVoiceLED(MATRIX_LED_R_1, 0, 125, 0);

    for(int i = 0; i < 3500 ; i++){

        soundLocalization.updateODAS();

		if (soundLocalization.getEnergy() > ENERGY_THRESHOLD)
			navigationObj.braitenberg(soundLocalization.getAngle(),  outputStream);
		else
			motorControl.changeMotorCommand(STOP);


	} // End of while loop
	outputStream.close();
	motorControl.changeMotorCommand(STOP);
	motorControl.resetMatrixVoiceLEDs();

/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	motorControl.setRightMotorSpeedDirection(0,1);
	motorControl.setLeftMotorSpeedDirection(0,1);


	std::cout << "End of main -------" << std::endl;

	return 0;
}
