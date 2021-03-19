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
#include "includes/camera.h"
#include "includes/LIDAR.h"

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
	MotorControl motorControl(&bus, &everloop, &everloop_image, &gpio);
	//ODAS soundLocalization(&bus, &everloop, &everloop_image);
	navigation navigation(&motorControl);


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
	************************  OUTOUT STREAM	    **********************************
	*****************************************************************************/

	//std::ofstream outputStream;
	//outputStream.open("./data/braitenbergMotorCommandsDummy.csv", std::ofstream::out | std::ofstream::trunc);
	//outputStream << "Left angle" << "," << "Activation output left" << "," << "Right angle" << "," << "Activation output right" << std::endl;

	/*****************************************************************************
	************************   CREATE THREADS	 *********************************
	*****************************************************************************/

	//std::thread threadOdas(&ODAS::updateODAS,	// the pointer-to-member
		//&soundLocalization);				// the object, could also be a pointer
							// the argument

    LIDAR lidar;
	std::thread threadLIDAR(&LIDAR::LIDARScan,
		&lidar);




	//Vision vision;

	char k;



	//while(true){
	for (int i = 0; i < 1000; i++) {
		rplidar_response_measurement_node_hq_t closestNode = lidar.readScan();
		std::cout << "Nearest distance to obstacle: " << closestNode.dist_mm_q2 /4.0f<< " Angle: "<< closestNode.angle_z_q14 * 90.f / (1 << 14) << std::endl;

		usleep(100000);

	//	//odas.updateODAS();
	//	//motor_control.setMatrixVoiceLED(MATRIX_LED_L_9, MAX_BRIGHTNESS, 0, 0);


	//	if (soundLocalization.getEnergy() > ENERGY_THRESHOLD) {
	//		navigation.braitenberg(soundLocalization.getSoundAngle(), outputStream);
	//	}
	//	else {
	//		motorControl.changeMotorCommand(STOP); //STOPS ALL MOTORS
	//	}

	//	vision.updateCamera();
	//	k = cv::waitKey(10);
	//	if (k == 27) //27 = 'ESC'
	//		break;
    }

/*********************************   END OF CONTROLLER LOOP   *********************************/

	motorControl.changeMotorCommand(STOP);		//STOP ALL MOTORS
	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS
	//vision.releaseCamera();						//Release camera resources

	//Test flag
	std::cout << "End of main -------" << std::endl;

	//threadOdas.join();
	//std::cout << "Odas thread joined" << std::endl;
	lidar.ctrlc();
	threadLIDAR.~thread();

	std::cout << "LIDAR thread terminated!" << std::endl;

	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS

	//outputStream.close();
	//vision.releaseCamera();
	std::cout << "End of main -------" << std::endl;

	return 0;
}
