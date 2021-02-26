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

#include <matrix_hal/everloop.h>
#include <matrix_hal/everloop_image.h>
#include <matrix_hal/gpio_control.h>
#include <matrix_hal/matrixio_bus.h>





namespace hal = matrix_hal;

// ENERGY_COUNT : Number of sound energy slots to maintain.
#define ENERGY_COUNT 36
// MAX_VALUE : controls smoothness
#define MAX_VALUE 200
// INCREMENT : controls sensitivity
#define INCREMENT 20
// DECREMENT : controls delay in the dimming
#define DECREMENT 1
// MAX_BRIGHTNESS: Filters out low energy
#define MIN_THRESHOLD 10
// MAX_BRIGHTNESS: 0 - 255
#define MAX_BRIGHTNESS 50
///**********************************************************************/
// GPIO via Matrix Voice
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

#define REVERSE                 -1
#define FORWARD                  1
#define LEFTTURN                 2
#define RIGHTTURN                3

using namespace std;
//using namespace cv;

// Initialise Matrix Voice GPIO pins
void initGPIOPins(matrix_hal::GPIOControl* gpio)
{

        gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA,1); //Pin mode as output
        gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA,1); // Pin function as PWM
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1,1);
        gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2,1);

        gpio->SetMode(TB6612_LEFT_MOTOR_PWMB,1); //Pin mode as output
        gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB,1); // Pin function as PWM
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN1,1);
        gpio->SetMode(TB6612_LEFT_MOTOR_BIN2,1);
}


// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,0); // Rotate left motor clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1,1); // Rotate left motor counter-clockwise
                gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
       if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
        if (dir <= 0) // Reverse
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,0); // Rotate right motor counter-clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,1);
        }
        if ( (dir > 0)  || (dir >= 1) ) // Forward
        {
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1,1); // Rotate right motor clockwise
                gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2,0);
        }

        // Set motor speed via PWM signal (min. = 0, max. = 100)
        if (speed > 100)
                speed = 100;
        if (speed < 0)
                speed = 0;

        gpio->SetPWM(1000,speed,TB6612_RIGHT_MOTOR_PWMA);
}

// Display a rainbow animation on the Matrix Voice LEDs
void startupShowLEDRainbow(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image)
{
        // Variables used for sine wave rainbow logic
        float counter = 0;
        const float freq = 0.375;

        // For each led in everloop_image.leds, set led value
        for (matrix_hal::LedValue &led : everloop_image->leds)
        {
                // Sine waves 120 degrees out of phase for rainbow
                led.red = (std::sin(freq * counter + (M_PI / 180 * 240)) * 155 + 100) / 10;
                led.green = (std::sin(freq * counter + (M_PI / 180 * 120)) * 155 + 100) / 10;
                led.blue = (std::sin(freq * counter + 0) * 155 + 100) / 10;
                counter = counter + 1.01;
                // Updates the LEDs
                everloop->Write(everloop_image);
                usleep(40000);
        }

        usleep(460000);

        // For each led in everloop_image.leds, set led value to 0
        for (matrix_hal::LedValue &led : everloop_image->leds)
        {
                // Turn off Everloop
                led.red = 0;
                led.green = 0;
                led.blue = 0;
                led.white = 0;
        }

        // Updates the Everloop on the MATRIX device
        everloop->Write(everloop_image);
}
// Set individual LEDs on the Matrix Voice
void setMatrixVoiceLED(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image,int ledn, int r, int g, int b)
{
       for (int i = 0; i < 18; i++)
        {
                if (i == ledn)
                {
                        everloop_image->leds[ledn].red = r;
                        everloop_image->leds[ledn].green = g;
                        everloop_image->leds[ledn].blue = b;
                        everloop_image->leds[ledn].white = 0;
                }
        }
        everloop->Write(everloop_image);
}



///**********************************************************************/
double x, y, z, E;
int energy_array[ENERGY_COUNT];
const double leds_angle_mcreator[35] = {
    170, 159, 149, 139, 129, 118, 108, 98,  87,  77,  67,  57,
    46,  36,  26,  15,  5,   355, 345, 334, 324, 314, 303, 293,
    283, 273, 262, 252, 242, 231, 221, 211, 201, 190, 180};

const double led_angles_mvoice[18] = {170, 150, 130, 110, 90,  70,
                                      50,  30,  10,  350, 330, 310,
                                      290, 270, 250, 230, 210, 190};

void increase_pots() {
  // Convert x,y to angle. TODO: See why x axis from ODAS is inverted
  double angle_xy = fmodf((atan2(y, x) * (180.0 / M_PI)) + 360, 360);
  // Convert angle to index
  int i_angle = angle_xy / 360 * ENERGY_COUNT;  // convert degrees to index
  // Set energy for this angle
  energy_array[i_angle] += INCREMENT * E;
  // Set limit at MAX_VALUE
  energy_array[i_angle] =
      energy_array[i_angle] > MAX_VALUE ? MAX_VALUE : energy_array[i_angle];
}

void decrease_pots() {
  for (int i = 0; i < ENERGY_COUNT; i++) {
    energy_array[i] -= (energy_array[i] > 0) ? DECREMENT : 0;
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
    } else if (type != json_type_object) {
    } else {
      json_parse(jvalue);
    }
  }
}

void json_parse(json_object *jobj) {
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
        } else if (!strcmp(key, "y")) {
          y = json_object_get_double(val);
        } else if (!strcmp(key, "z")) {
          z = json_object_get_double(val);
        } else if (!strcmp(key, "E")) {
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
    }
  }
}


/*******/

std::vector<int> getEnergyArray()
{
	std::vector<int> energy_vector(ENERGY_COUNT);
	for (size_t i = 0; i < ENERGY_COUNT; i++)
	{
		energy_vector.push_back(energy_array[i]);
	}
	return energy_vector;
}

double getSoundAngle() {
	int largest_element_index;
	int largest_element = -1;
	for (size_t i = 0; i < ENERGY_COUNT; i++)
	{
		if (energy_array[i] > largest_element)
		{
			largest_element = energy_array[i];
			largest_element_index = i;
		}
	}
	return (largest_element_index * 360 / ENERGY_COUNT);
	//int index_pots = led_angle * ENERGY_COUNT / 360;
}

/******/


int main(int argc, char *argv[]) {

/*****************************************************************************
*********************   INITIALISE MATRIX VOICE DEVICE   *********************
*****************************************************************************/

        // Create MatrixIOBus object for hardware communication
        matrix_hal::MatrixIOBus bus;
        // Initialize bus and exit program if error occurs

        if(!bus.Init())
            return false;

        // Holds the number of LEDs on MATRIX device
        int ledCount = bus.MatrixLeds();
        // Create EverloopImage object, with size of ledCount
        matrix_hal::EverloopImage everloop_image(ledCount);
        // Create Everloop object
        matrix_hal::Everloop everloop;
        // Set everloop to use MatrixIOBus bus
        everloop.Setup(&bus);
        // Create GPIOControl object
        matrix_hal::GPIOControl gpio;
        // Set gpio to use MatrixIOBus bus
        gpio.Setup(&bus);

	// Display rainbow animation
	startupShowLEDRainbow(&everloop, &everloop_image);

///*********************************   DONE   *********************************/

/*****************************************************************************
************************   INITIALISE MOTOR CONTROL   ************************
*****************************************************************************/

	/* Initialise Matrix Voice GPIO pins */
        initGPIOPins( &gpio);

              int motorR = 0, motorL = 0, motorLDir = FORWARD, motorRDir = FORWARD;
              	// Stop all motors
        setRightMotorSpeedDirection(&gpio,0,1);
        setLeftMotorSpeedDirection(&gpio,0,1);


/*********************************   DONE   *********************************/


/***********Set up connection to ODAS ****************/

  // Everloop Initialization
  //hal::MatrixIOBus bus;
  if (!bus.Init())
    return false;
  hal::EverloopImage image1d(bus.MatrixLeds());
  //hal::Everloop everloop;
  everloop.Setup(&bus);

  // Clear all LEDs
  for (hal::LedValue &led : image1d.leds) {
    led.red = 0;
    led.green = 0;
    led.blue = 0;
    led.white = 0;
  }
  everloop.Write(&image1d);

  char verbose = 0x00;

  int server_id;
  struct sockaddr_in server_address;
  int connection_id;
  char *message;
  int messageSize;

  int c;
  unsigned int portNumber = 9001;
  const unsigned int nBytes = 10240;

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

  printf("Receiving data........... \n\n");

  /* My variables */
  int angle = -1, currentMax = -1;
  std::vector<int> angleVec; //Contains the angle values from MatrixVoice
  int action; //Use to define behavior

  for(int i=0; i < bus.MatrixLeds(); i++){
    angleVec.push_back(-1);
  }

  /*************** Main loop ***************/


  while ((messageSize = recv(connection_id, message, nBytes, 0)) > 0) {
    //std::cout << "Matrix LEDs: " << bus.MatrixLeds() << std::endl;


    message[messageSize] = 0x00;

    //printf("message: %s\n\n", message);
    json_object *jobj = json_tokener_parse(message);
    json_parse(jobj);

    setRightMotorSpeedDirection(&gpio, 0, 1);
    setLeftMotorSpeedDirection(&gpio, 0, 1);


    /************* Sound detection ***************/
    for (int i = 0; i < bus.MatrixLeds(); i++) {
      // led index to angle
      int led_angle = bus.MatrixName() == hal::kMatrixCreator
                          ? leds_angle_mcreator[i]
                          : led_angles_mvoice[i];



      // Convert from angle to pots index
      int index_pots = led_angle * ENERGY_COUNT / 360;
      // Mapping from pots values to color
      int color = energy_array[index_pots] * MAX_BRIGHTNESS / MAX_VALUE;
      // Removing colors below the threshold
      color = (color < MIN_THRESHOLD) ? 0 : color;

      image1d.leds[i].red = 0;
      image1d.leds[i].green = color;
      image1d.leds[i].blue = 0;
      image1d.leds[i].white = 0;

      if(color > MIN_THRESHOLD)
            angleVec[i] = led_angle;
      else
            angleVec[i] = -1;
    }

    currentMax = -1;
    for(int i = 0; i < angleVec.size() ; i++){
        if( currentMax < angleVec[i] )
            currentMax = angleVec[i];
    }
    angle = currentMax;
    if(angle != -1)
    std::cout << "Angle from currentMax: "<< angle<< "Angle from getAngle()" << getSoundAngle() << std::endl;

    /************* Behavior control ***************/
/*
    switch (action) {

        case FORWARD :
            std::cout << "Go straight! " << std::endl;
            motorR = 25, motorL = 25; //Changed from 0 on both
            motorLDir = 1, motorRDir = 1;

            if (angle > 190 && angle < 360)
                action = LEFTTURN;

            else if (angle < 190 && angle > 0)
                action = RIGHTTURN;

            break;

        case LEFTTURN :
            std::cout << "Go left! \n";
            motorR = 25, motorL = 20;
            motorLDir = -1;

            if(angle == 190)
                action = FORWARD;
            else if (angle < 190 && angle > 0)
                action = RIGHTTURN;

            break;

        case RIGHTTURN :
            std::cout << "Go right! \n";
            motorR = 20, motorL = 25;
            motorRDir = -1;

            if(angle == 190)
                action = FORWARD;
            else if (angle > 190 && angle < 360)
                action = LEFTTURN;
            break;

        case -1:
            if (angle > 190 && angle < 360)
                action = LEFTTURN;
            else if(angle == 190)
                action = FORWARD;

            else if (angle < 190 && angle > 0)
                action = RIGHTTURN;
            break;

            default :
            std::cout << "Default case, something went wrong. \n";
            break;
    }
*/
    if(angle != -1)
    cout << "Direction of sound: " << angle << "\n";

    setRightMotorSpeedDirection(&gpio, motorR, motorRDir);
    setLeftMotorSpeedDirection(&gpio, motorL, motorLDir);

    everloop.Write(&image1d);
  }
}
