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
//#include "includes/ODAS.h"


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



#define ENERGY_COUNT 36		// ENERGY_COUNT : Number of sound energy slots to maintain.
#define MAX_VALUE 200		// MAX_VALUE : controls smoothness
#define INCREMENT 20		// INCREMENT : controls sensitivity
#define DECREMENT 1			// DECREMENT : controls delay in the dimming
#define MIN_THRESHOLD 10//10	// MAX_BRIGHTNESS: Filters out low energy
#define MAX_BRIGHTNESS 50	// MAX_BRIGHTNESS: 0 - 255
#define ENERGY_THRESHOLD 30





using namespace std;
//using namespace cv;

double x, y, z, E;
int energyArray[ENERGY_COUNT];
std::vector<int> angleVec;
const double led_angles_mvoice[18] = { 170, 150, 130, 110, 90,  70,
								  50,  30,  10,  350, 330, 310,
								  290, 270, 250, 230, 210, 190 }; //LED angles for MATRIX Voice
const double leds_angle_mcreator[35] = { 170, 159, 149, 139, 129, 118, 108, 98,  87,  77,  67,  57,
										46,  36,  26,  15,  5,   355, 345, 334, 324, 314, 303, 293,
										283, 273, 262, 252, 242, 231, 221, 211, 201, 190, 180 };
int ledCount;
matrix_hal::MatrixIOBus bus;				// Create MatrixIOBus object for hardware communication
matrix_hal::EverloopImage* image1d;			// Create EverloopImage object "image1d", with size of ledCount
matrix_hal::Everloop* everloop;				// Create Everloop object

//Connection variables
char verbose = 0x00;
int server_id;
struct sockaddr_in server_address;
int connection_id;
char *message;
int messageSize;
int c;
unsigned int portNumber = 9001;
const unsigned int nBytes = 10240;

void increase_pots() {
	// Convert x,y to angle. TODO: See why x axis from ODAS is inverted
	double angle_xy = fmodf((atan2(y, x) * (180.0 / M_PI)) + 360, 360);
	// Convert angle to index
	//double d_angle = (angle_xy / 360.0 * (double)ENERGY_COUNT);
	//int i_angle = d_angle;
	int i_angle = (angle_xy / 360 * ENERGY_COUNT);  // convert degrees to index
	// Set energy for this angle
	energyArray[i_angle] += INCREMENT * E;
	// Set limit at MAX_VALUE
	energyArray[i_angle] =
		energyArray[i_angle] > MAX_VALUE ? MAX_VALUE : energyArray[i_angle];
}

void decrease_pots() {
	for (int i = 0; i < ENERGY_COUNT; i++) {
		energyArray[i] -= (energyArray[i] > 0) ? DECREMENT : 0;
	}
}

void json_parse_array(json_object *jobj, char *key) {
	// Forward Declaration
	void json_parse(json_object * jobj);
	enum json_type type;
	json_object *jarray = jobj;
	if (key) {
		if (json_object_object_get_ex(jobj, key, &jarray) == false) {
			printf("Error parsing json object\n");
			return;
		}
	}

	int arraylen = json_object_array_length(jarray);
	int i;
	json_object *jvalue;

	for (i = 0; i < arraylen; i++) {
		jvalue = json_object_array_get_idx(jarray, i);
		type = json_object_get_type(jvalue);

		if (type == json_type_array) {
			json_parse_array(jvalue, NULL);
		}
		else if (type != json_type_object) {
		}
		else {
			json_parse(jvalue);
		}
	}
}

void json_parse(json_object * jobj)
{
	enum json_type type;
	unsigned int count = 0;
	decrease_pots();
	json_object_object_foreach(jobj, key, val) {
		type = json_object_get_type(val);
		switch (type) {
		case json_type_boolean:
			break;
		case json_type_double:
			if (!strcmp(key, "x")) {
				x = json_object_get_double(val);
			}
			else if (!strcmp(key, "y")) {
				y = json_object_get_double(val);
			}
			else if (!strcmp(key, "z")) {
				z = json_object_get_double(val);
			}
			else if (!strcmp(key, "E")) {
				E = json_object_get_double(val);
			}
			increase_pots();
			count++;
			break;
		case json_type_int:
			break;
		case json_type_string:
			break;
		case json_type_object:
			if (json_object_object_get_ex(jobj, key, &jobj) == false) {
				printf("Error parsing json object\n");
				return;
			}
			json_parse(jobj);
			break;
		case json_type_array:
			json_parse_array(jobj, key);
			break;
		case json_type_null: //This type wasnt supported in original odas implementation, fixed warning
			break;
		}
	}
}

void setupOdas() {
	// Everloop Initialization
	// Initialize bus and exit program if error occurs
	if (!bus.Init())
		throw("Bus Init failed");
	//return false;

	// Holds the number of LEDs on MATRIX device
	ledCount = bus.MatrixLeds();
	// Create EverloopImage object, with size of ledCount
	image1d = new matrix_hal::EverloopImage(ledCount);

	// Create Everloop object
	everloop = new matrix_hal::Everloop;
	// Set everloop to use MatrixIOBus bus
	everloop->Setup(&bus);

	// Clear all LEDs
	for (matrix_hal::LedValue &led : image1d->leds) {
		led.red = 0;
		led.green = 0;
		led.blue = 0;
		led.white = 0;
	}
	everloop->Write(image1d);

	//Test values - 25/02 problems with not all matrix voice LEDs lighting up as expected
	//printf("\nDefines: ENERGY_COUNT:%d - MAX_BRIGHTNESS:%d - MAX_VALUE%d - MIN_THRESHOLD:%d - INCREMENT:%d",ENERGY_COUNT, MAX_BRIGHTNESS, MAX_VALUE, MIN_THRESHOLD,INCREMENT);
	printf("\nbus.MatrixLeds(): %d --------------\n", bus.MatrixLeds());


	server_id = socket(AF_INET, SOCK_STREAM, 0);

	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_address.sin_port = htons(portNumber);

	printf("Binding socket........... ");
	fflush(stdout);
	bind(server_id, (struct sockaddr *)&server_address, sizeof(server_address));
	printf("[OK]\n");

	printf("Listening socket......... ");
	fflush(stdout);
	listen(server_id, 1);
	printf("[OK]\n");

	printf("Waiting for connection in port %d ... ", portNumber);
	fflush(stdout);
	connection_id = accept(server_id, (struct sockaddr *)NULL, NULL);
	printf("[OK]\n");

	message = (char *)malloc(sizeof(char) * nBytes);

    for(int i=0; i < bus.MatrixLeds(); i++){
        angleVec.push_back(-1);
    }

	printf("Receiving data........... \n\n");
}

//double getSoundAngle() {
//	int largest_element_index;
//	int largest_element = -1;
//	for (size_t i = 0; i < ENERGY_COUNT; i++)
//	{
//		if (energy_array[i] > largest_element)
//		{
//			largest_element = energy_array[i];
//			largest_element_index = i;
//		}
//	}
//	return (largest_element_index * 360 / ENERGY_COUNT);
//}

void updateODAS() {
	//while ((messageSize = recv(connection_id, message, nBytes, 0)) > 0) {
	if((messageSize = recv(connection_id, message, nBytes, 0)) > 0) {
		message[messageSize] = 0x00;

		// printf("message: %s\n\n", message);
		json_object *jobj = json_tokener_parse(message);
		json_parse(jobj);

		for (int i = 0; i < bus.MatrixLeds(); i++) {
			// led index to angle
			int led_angle = bus.MatrixName() == matrix_hal::kMatrixCreator
				? leds_angle_mcreator[i]
				: led_angles_mvoice[i];
			//int led_angle = led_angles_mvoice[i];
			// Convert from angle to pots index
			int index_pots = led_angle * ENERGY_COUNT / 360;
			// Mapping from pots values to color
			int color = energyArray[index_pots] * MAX_BRIGHTNESS / MAX_VALUE;
			// Removing colors below the threshold
			color = (color < MIN_THRESHOLD) ? 0 : color;

			image1d->leds[i].red = 0;
			image1d->leds[i].green = 0;//0;
			image1d->leds[i].blue = color;//color;
			image1d->leds[i].white = 0;

//            if(color > MIN_THRESHOLD)
//                angleVec[i] = led_angle;
//            else
//                angleVec[i] = -1;
        }

//    currentMax = -1;
//    for(unsigned int i = 0; i < angleVec.size() ; i++){
//        if( currentMax < angleVec[i] )
//            currentMax = angleVec[i];
//    }

//    if(currentMax != -1 && getSoundAngle() > 0 && currentMax != prevMax){
//        prevMax = angle;
//        angle = currentMax;
//        //std::cout << "Angle from currentMax: "<< angle<< ", Angle from getAngle(): " << getSoundAngle() << std::endl;
//    }
		everloop->Write(image1d);
	}
}


void getSoundInformation(int& angle, int& energy) {
	int largestElementIndex;
	int largestElement = -1;
	for (size_t i = 0; i < ENERGY_COUNT; i++)
	{
		if (energyArray[i] > largestElement)
		{
			largestElement = energyArray[i];
			largestElementIndex = i;
		}
	}
	if (largestElement != -1)
	{
		angle = (largestElementIndex * 360 / ENERGY_COUNT);
		energy = largestElement;
	}
}

/* Values for testing Braitenberg function */
double test_angle = 270;
double ICO_weight = 1;

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
	setupOdas();
	//ODAS odas;
	//Vision vision;

	motorControl.changeMotorCommand(STOP);
/*********************************   DONE   *********************************/


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
	int angle = -1, prevAngle = -1, energy = -1;

    //odas.updateODAS();
    //for(int i = 0; i < 15;i++)	{
    while(true){

        updateODAS();
		getSoundInformation(angle, energy);

		if (angle != prevAngle) {
			std::cout << "Angle: " << angle << " Energy: " << energy << std::endl;
			prevAngle = angle;
		}

		if (energy > ENERGY_THRESHOLD)
			braitenberg(angle, &motorControl);
		else
			motorControl.changeMotorCommand(STOP);


	} // End of while loop

	motorControl.changeMotorCommand(STOP);
	motorControl.resetMatrixVoiceLEDs();

/*********************************   END OF CONTROLLER LOOP   *********************************/

	// Stop all motors
	motorControl.setRightMotorSpeedDirection(0,1);
	motorControl.setLeftMotorSpeedDirection(0,1);

	//Test flag
	std::cout << "End of main -------" << std::endl;

	return 0;
}
