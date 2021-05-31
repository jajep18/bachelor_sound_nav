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
	************************   INITIALISE CLASSES & THREADS **********************
	*****************************************************************************/
	MotorControl motorControl(&bus, &everloop, &everloop_image, &gpio);

    navigation navigation(&motorControl);

	ODAS soundLocalization(&bus, &everloop, &everloop_image);
	std::thread threadODAS(&ODAS::updateODAS, &soundLocalization);

	LIDAR lidar;
	std::thread threadLIDAR(&LIDAR::LIDARScan, &lidar);

	Vision vision;
	std::thread threadVision(&Vision::updateCamera, &vision);
    std::thread threadObjDetect(&Vision::YOLOProcess, &vision);


	/*****************************************************************************
	************************  OUTOUT STREAM	    **********************************
	*****************************************************************************/

	std::ofstream outputStream, outputStreamICO;
	outputStream.open("./data/dummy.csv", std::ofstream::out | std::ofstream::trunc);
    outputStreamICO.open("./data/dummyICO.csv", std::ofstream::out | std::ofstream::trunc);
	//outputStream << "Left angle" << "," << "Activation output left" << "," << "Right angle" << "," << "Activation output right" << std::endl;

	/*****************************************************************************
	************************  SETUP VARIABLES   **********************************
	*****************************************************************************/


	//Obstacle avoidance / ICO Learning
	double distToObstCurrent = 1000;			// Distance to closest obstacle on the track
	double angleToObst = 0;
	double distToObstPrev;						// Previous Distance to closest obstacle on the track


	double reflexDistToObstCurrent = 1000;
	double reflexAngleToObst = 0;
	double reflexDistToObstPrev = 0;			// Previous Distance to closest obstacle on the track
	double reflexDistToObstPrevPrev = 35.0;

	double distToDetectedObj = 999.0;





	//LIDAR stabilization to prevent wrong readings since first readings are 0
	rplidar_response_measurement_node_hq_t closest_node = lidar.readScan();
	std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	while (closest_node.dist_mm_q2 == 0) {
		closest_node = lidar.readScan();
		std::cout << " Angle: " << lidar.getCorrectedAngle(closest_node) << " Nearest dist: " << closest_node.dist_mm_q2 / 4.0f << " (Lidar stabilizing)" << std::endl;
	}

	/*********************************   CONTROLLER LOOP   *********************************/

	states CURRENT_STATE = WAIT;
    motorControl.setMatrixVoiceLED(MATRIX_LED_L_1 ,MAX_BRIGHTNESS,0,0);

	while (true) {
		rplidar_response_measurement_node_hq_t closestNode = lidar.readScan();
		rplidar_response_measurement_node_hq_t closestNodeReflex = lidar.readScanReflex();
		rplidar_response_measurement_node_hq_t objNode = lidar.readScanObjCheck();


		distToObstPrev		= distToObstCurrent;
		distToObstCurrent	= closestNode.dist_mm_q2 / 4.0f;
		angleToObst			= lidar.getCorrectedAngle(closestNode);

		reflexDistToObstPrevPrev    = reflexDistToObstPrev;
        reflexDistToObstPrev        = reflexDistToObstCurrent;
        reflexDistToObstCurrent     = closestNodeReflex.dist_mm_q2 / 4.0f;
        reflexAngleToObst           = lidar.getCorrectedAngle(closestNodeReflex);


        distToDetectedObj = objNode.dist_mm_q2/4.0f; //Object detection


        navigation.updateState(distToObstCurrent, reflexDistToObstCurrent, soundLocalization.getEnergy(), CURRENT_STATE, vision.getObject(), vision.getConfidence(), distToDetectedObj, soundLocalization.getAngle());
		switch (CURRENT_STATE)
		{
		case WAIT: //Cyan
			motorControl.changeMotorCommand(STOP, STOP, STOP);		//STOP ALL MOTORS

			motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, 0, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
			break;
		case NAVIGATE: //Blue
			navigation.braitenberg(soundLocalization.getAngle(), outputStream, 0, 0);

			motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, 0, 0, MAX_BRIGHTNESS);
			break;
		case AVOID: // Yellow

			navigation.obstacleAvoidance(angleToObst, soundLocalization.getAngle(), distToObstCurrent, distToObstPrev, outputStream);

			motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0);

			break;
		case REFLEX: //Red

			navigation.obstacleReflex(reflexAngleToObst, reflexDistToObstCurrent, reflexDistToObstPrev, reflexDistToObstPrevPrev);

			motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, MAX_BRIGHTNESS, 0, 0);
			break;
		case TARGET_FOUND:
			//Stop & check for correct target
			std::cout << "Target found !\n";
			motorControl.changeMotorCommand(STOP, STOP, STOP);		//STOP ALL MOTORS
			motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, 0, MAX_BRIGHTNESS, 0);


            navigation.manualInputSteering(&vision, outputStreamICO);
			break;
		case NAVIGATE_TO_PERSON: //White

			motorControl.changeMotorCommand(FORWARD,30,33);

            motorControl.setMatrixVoiceLED(MATRIX_LED_R_9, MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
			break;
		default:
			break;
		}


        usleep(100000);

        //Check Vision thread waitkey - exit or manual steering
        if(vision.inputKey == 112){ //112 = 'p'
            motorControl.setMatrixVoiceLED(MATRIX_LED_L_9,MAX_BRIGHTNESS,0,MAX_BRIGHTNESS); //Purple
            navigation.manualInputSteering(&vision, outputStreamICO);
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

	threadObjDetect.join();
	std::cout << "YOLO thread joined" << std::endl;


	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS


	std::cout << "End of main -------" << std::endl;

	return 0;
}
