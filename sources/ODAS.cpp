#include "../includes/ODAS.h"

void ODAS::increase_pots() {
	// Convert x,y to angle. TODO: See why x axis from ODAS is inverted
	double angle_xy = fmodf((atan2(y, x) * (180.0 / M_PI)) + 360, 360);
	//std::cout << "angle_xy: " <<angle_xy << std::endl;
	// Convert angle to index
	double d_angle = angle_xy / 360 * ENERGY_COUNT;  // convert degrees to index
	int i_angle = d_angle;
	//std::cout << "i_angle & d_anle: " << i_angle << d_angle <<std::endl;
	// Set energy for this angle
	energyArray[i_angle] += INCREMENT * E;
	// Set limit at MAX_VALUE
	energyArray[i_angle] =
		energyArray[i_angle] > MAX_VALUE ? MAX_VALUE : energyArray[i_angle];
}

void ODAS::decrease_pots() {
	for (int i = 0; i < ENERGY_COUNT; i++) {
		energyArray[i] -= (energyArray[i] > 0) ? DECREMENT : 0;
	}
}

//void ODAS::json_parse(json_object* jobj);

void ODAS::json_parse_array(json_object* jobj, char* key) {
	// Forward Declaration
	//void ODAS::json_parse2(json_object * jobj);
	enum json_type type;
	json_object* jarray = jobj;
	if (key) {
		if (json_object_object_get_ex(jobj, key, &jarray) == false) {
			printf("Error parsing json object\n");
			return;
		}
	}

	int arraylen = json_object_array_length(jarray);
	int i;
	json_object* jvalue;

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

void ODAS::json_parse(json_object* jobj)
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
        case json_type_null:
            break;
		}
	}
}

ODAS::ODAS(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* image1d_) {

//	// Everloop Initialization
//	// Initialize bus and exit program if error occurs
//	if (!bus.Init())
//		throw("Bus Init failed");
//
//// Holds the number of LEDs on MATRIX device
//	ledCount = bus.MatrixLeds();
//	//std::cout << "\n bus.MatrixLeds: " <<  bus.MatrixLeds() <<std::endl;
//	// Create EverloopImage object, with size of ledCount
//	image1d = new matrix_hal::EverloopImage(ledCount);
//
//	// Create Everloop object
//	everloop = new matrix_hal::Everloop;
//	// Set everloop to use MatrixIOBus bus
//	everloop->Setup(&bus);
/******************************************/
	bus = bus_;
	everloop = everloop_;
	image1d = image1d_;
/******************************************/
	 //Clear all LEDs
	for (matrix_hal::LedValue& led : image1d->leds) {
		led.red = 0;
		led.green = 0;
		led.blue = 0;
        led.white = 0;
	}
	everloop->Write(image1d);

	//Test values - 25/02 problems with not all matrix voice LEDs lighting up as expected
	//printf("\nDefines: ENERGY_COUNT:%d - MAX_BRIGHTNESS:%d - MAX_VALUE%d - MIN_THRESHOLD:%d - INCREMENT:%d",ENERGY_COUNT, MAX_BRIGHTNESS, MAX_VALUE, MIN_THRESHOLD,INCREMENT);
	printf("\nbus.MatrixLeds(): %d --------------\n", bus->MatrixLeds());


	server_id = socket(AF_INET, SOCK_STREAM, 0);

	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_address.sin_port = htons(portNumber);

	printf("Binding socket........... ");
	fflush(stdout);
	bind(server_id, (struct sockaddr*)&server_address, sizeof(server_address));
	printf("[OK]\n");

	printf("Listening socket......... ");
	fflush(stdout);
	listen(server_id, 1);
	printf("[OK]\n");

	printf("Waiting for connection in port %d ... ", portNumber);
	fflush(stdout);
	connection_id = accept(server_id, (struct sockaddr*)NULL, NULL);
	printf("[OK]\n");

	message = (char *)malloc(sizeof(char) * nBytes);



	printf("Receiving data........... \n\n");

}

ODAS::~ODAS() {}

void ODAS::updateODAS() {

	while((messageSize = recv(connection_id, message, nBytes, 0)) > 0) {

			message[messageSize] = 0x00;

			//printf("message: %s\n\n", message);
			json_object* jobj = json_tokener_parse(message);
			json_parse(jobj);

			for (int i = 0; i < bus->MatrixLeds(); i++) {

				int led_angle = led_angles_mvoice[i];								//Define angle for `
				int index_pots = led_angle * ENERGY_COUNT / 360;					// Convert from angle to pots index
				int color = energyArray[index_pots] * MAX_BRIGHTNESS / MAX_VALUE; 	// Mapping from pots values to color+
				color = (color < MIN_THRESHOLD) ? 0 : color; 						// Removing colors below the threshold


//				if(energyArray[index_pots] > 100)
//				std::cout << "\nLED nr " << i+1 << ". Energy array value: " << energyArray[index_pots] << " Index pots value: "<< index_pots << std::endl;
//
//				if(color > 0)
//				std::cout << "\nLED nr " << i+1 << ". Color value: "<< color << std::endl;

//				image1d->leds[i].red = 0;
				image1d->leds[i].green = color;
//				image1d->leds[i].blue = 0;
//				image1d->leds[i].white = 0;

			}

			everloop->Write(image1d); //Writes to LEDs

			updateSoundInformation(); //Updates sound angle and energy
		}

}



std::vector<int> ODAS::getEnergyArray()
{
	std::vector<int> energy_vector(ENERGY_COUNT);
	for (size_t i = 0; i < ENERGY_COUNT; i++)
	{
		energy_vector.push_back(energyArray[i]);
	}
	return energy_vector;
}

double ODAS::getSoundAngle() {
	int largest_element_index;
	int largest_element = -1;
	for (size_t i = 0; i < ENERGY_COUNT; i++)
	{
		if (energyArray[i] > largest_element)
		{
			largest_element = energyArray[i];
			largest_element_index = i;
		}
	}
	return (largest_element_index * 360 / ENERGY_COUNT);
	//int index_pots = led_angle * ENERGY_COUNT / 360;
}

void ODAS::updateSoundInformation() {
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

	//Lock angle and sound mutex until end of scope
	std::lock_guard<std::mutex> guard(angleEnergyMutex);

	if (largestElement != -1)
	{
		angle = (largestElementIndex * 360 / ENERGY_COUNT);
		energy = largestElement;
	}

	if (angle != prevAngle) {
        if(energy > ENERGY_THRESHOLD)
            std::cout << "Angle: " << angle << " Energy: " << energy << std::endl;
		prevAngle = angle;
	}
}


int ODAS::getAngle() {

	int tempAngle;

	while (true) {
		// try to lock mutex to modify 'angle'
		if (angleEnergyMutex.try_lock()) {
			tempAngle = angle;
			angleEnergyMutex.unlock();
			break;
		}
    }
	return tempAngle;

}

int ODAS::getEnergy(){

    int tempEnergy;

	while (true) {
		// try to lock mutex to modify 'energy'
		if (angleEnergyMutex.try_lock()) {
			tempEnergy = energy;
			angleEnergyMutex.unlock();
			break;
		}
	}
	return tempEnergy;

}




