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

	double wReflexVar = 1.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double wReflexConst = 1.0;		//

	double reflexLearningRate = 10;	// Learning rate for reflex µ
	double vLearning = 0.0; 		// Velocity to add to the initial velocity
	int reflexCounter = 0;





	//while(true){
	while (true) {
		rplidar_response_measurement_node_hq_t closestNode = lidar.readScan();
		std::cout << "Angle: "<< lidar.getCorrectedAngle(closestNode) << " Nearest distance to obstacle: " << closestNode.dist_mm_q2 /4.0f << std::endl;

		//usleep(100000);

        distToObstPrevPrev	= distToObstPrev;
		distToObstPrev		= distToObstCurrent;
		distToObstCurrent	= closestNode.dist_mm_q2 / 4.0f;
		angleToObst			= lidar.getCorrectedAngle(closestNode);

        if (distToObstCurrent < REFLEX_THRESHOLD){
            //LEFT OR RIGHT REFLEX DODGING
            if (angleToObst <= 180){ //RIGHT SIDE OBSTACLE
                double angleNorm = (angleToObst - 90) / 90;
                motorControl.setRightMotorSpeedDirection(navigation.activationFunction(angleNorm));
                motorControl.setLeftMotorSpeedDirection(navigation.activationFunction(-angleNorm));

            }
            else { // angleToObst > 180 //LEFT SIDE OBSTACLE
                double angleNorm = (90 - (angleToObst - 180)) / 90;
                motorControl.setRightMotorSpeedDirection(navigation.activationFunction(-angleNorm));
                motorControl.setLeftMotorSpeedDirection(navigation.activationFunction(angleNorm));
            }
            //Update weight used for vLearning
            wReflexVar = wReflexVar + reflexLearningRate * (distToObstCurrent / REFLEX_THRESHOLD) * (distToObstPrev - distToObstPrevPrev) / REFLEX_THRESHOLD;
            reflexCounter += 1;
        }
        else if (soundLocalization.getEnergy() > ENERGY_THRESHOLD) {
            if (distToObstCurrent < AVOIDANCE_THRESHOLD){
                vLearning = (distToObstCurrent / REFLEX_THRESHOLD) * wReflexVar + (distToObstPrev / REFLEX_THRESHOLD) * wReflexConst;
                if (angleToObst <= 180){  //RIGHT SIDE OBSTACLE
                    navigation.braitenberg(soundLocalization.getAngle(), outputStream, 0, vLearning);
                }else { // angleToObst > 180 //LEFT SIDE OBSTACLE
                    navigation.braitenberg(soundLocalization.getAngle(), outputStream, vLearning, 0);
                }
            }
            else{
                navigation.braitenberg(soundLocalization.getAngle(), outputStream, 0, 0);
            }
        }
        else {
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
            std::cout << "done"  << std::endl;
            break;
        }
    }

/*********************************   END OF CONTROLLER LOOP   *********************************/

	motorControl.changeMotorCommand(STOP);		//STOP ALL MOTORS
	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS
	outputStream.close();

	//Test flag
	std::cout << "End of main -------" << std::endl;


	lidar.ctrlc(0); //Takes an int
	threadLIDAR.join();

	std::cout << "LIDAR thread joined" << std::endl;


	threadODAS.join();
	std::cout << "ODAS thread joined" << std::endl;



	motorControl.resetMatrixVoiceLEDs();		//RESET ALL LEDS


	return 0;
}
