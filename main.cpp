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

    navigation navigation(&motorControl);

	ODAS soundLocalization(&bus, &everloop, &everloop_image);
	std::thread threadODAS(&ODAS::updateODAS, &soundLocalization);

	Vision vision;
	std::thread threadVision(&Vision::updateCamera, &vision);

    LIDAR lidar;
	std::thread threadLIDAR(&LIDAR::LIDARScan, &lidar);

	/*****************************************************************************
	************************   ICO LEARNING   ************************************
	*****************************************************************************/
	/*****************************************************************************
	************************   CONTROLLER LOOP   *********************************
	*****************************************************************************/

	/*****************************************************************************
	************************  OUTOUT STREAM	    **********************************
	*****************************************************************************/

	std::ofstream outputStream;
	outputStream.open("./data/dummy.csv", std::ofstream::out | std::ofstream::trunc);
	outputStream << "Left angle" << "," << "Activation output left" << "," << "Right angle" << "," << "Activation output right" << std::endl;



	//Obstacle avoidance / ICO Learning
	double distToObstCurrent = 1000;		// Distance to closest obstacle on the track
	double angleToObst = 0;
	double distToObstPrev;				// Previous Distance to closest obstacle on the track

	double distToObstPrevPrev = 35.0;	// Previous Previus Distance to closest obstacle on the track


	//LIDAR stabilization to prevent wrong readings since first readings are 0
	rplidar_response_measurement_node_hq_t closest_node = lidar.readScan();
	std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	while (closest_node.dist_mm_q2 == 0) {
		closest_node = lidar.readScan();
		std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	}

	/*********************************   CONTROLLER LOOP   *********************************/

	while (true) {
		rplidar_response_measurement_node_hq_t closestNode = lidar.readScan();
		std::cout << "Angle: "<< lidar.getCorrectedAngle(closestNode) << " Nearest distance to obstacle: " << closestNode.dist_mm_q2 /4.0f << std::endl;

		//usleep(100000);

        distToObstPrevPrev	= distToObstPrev;
		distToObstPrev		= distToObstCurrent;
		distToObstCurrent	= closestNode.dist_mm_q2 / 4.0f;
		angleToObst			= lidar.getCorrectedAngle(closestNode);

        if (distToObstCurrent < REFLEX_THRESHOLD){ //Reflex avoidance

			navigation.obstacleReflex(angleToObst, distToObstCurrent, distToObstPrev, distToObstPrevPrev);

        }
        else if (soundLocalization.getEnergy() > ENERGY_THRESHOLD) {
            if (distToObstCurrent < AVOIDANCE_THRESHOLD){ //Obstacle avoidance - obstacle inside avoidance threshold

				navigation.obstacleAvoidance(angleToObst, distToObstCurrent, distToObstPrev);

            }
            else{	//Braitenberg
                navigation.braitenberg(soundLocalization.getAngle(), outputStream, 0, 0);
            }
        else { // If no sound
            motorControl.changeMotorCommand(STOP, STOP, STOP);		//STOP ALL MOTORS
        }

        usleep(100000);

        //Check Vision thread waitkey - exit or manual steering
        if(vision.inputKey == 112){ //112 = 'p'
            navigation.manualInputSteering(&vision);
        }
        if(vision.inputKey == 27){ //27 = 'ESC'
            std::cout << "Vision thread joining...";
            threadVision.join();
            std::cout << " Done"  << std::endl;
            break;
        }
    }

/*********************************   END OF CONTROLLER LOOP   *********************************/

	motorControl.changeMotorCommand(STOP);		//STOP ALL MOTORS
	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS
	outputStream.close();


	lidar.ctrlc();
	threadLIDAR.join();

	std::cout << "LIDAR thread joined" << std::endl;


	threadODAS.join();
	std::cout << "ODAS thread joined" << std::endl;



	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS

		//Test flag
	std::cout << "End of main -------" << std::endl;

	return 0;
}
