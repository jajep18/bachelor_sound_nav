#include "../includes/navigation.h"

navigation::navigation(MotorControl* motorControl_)
	: motorControl{ motorControl_ }{
	std::cout << "Navigation ready!\n";
}

double navigation::activationFunction(double input) {
	/* Test functions */
	return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid

}

void navigation::braitenberg(double angle, std::ofstream& outputBraitenberg) { //Braitenberg aggression vehicle

	// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

	//Calculate motorspeed using activation function
	motorSpeedL = activationFunction(angleL) + VELOCITY_OFFSET;
	motorSpeedR = activationFunction(angleR) + VELOCITY_OFFSET;

	std::cout << "Motorspeed left: " << motorSpeedL << ". Motorspeed right: " << motorSpeedR << std::endl;

	//motorControl->changeMotorCommand(motorCommand, motorSpeedL, motorSpeedR);
	motorControl->setLeftMotorSpeedDirection(motorSpeedL, FORWARD);
	motorControl->setRightMotorSpeedDirection(motorSpeedR,FORWARD);

	outputBraitenberg << angleL << "," << motorSpeedL << "," << angleR << "," << motorSpeedR << std::endl;
}

void navigation::navigationICO(double angle, double w_A) {

		// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

//	motor_control->setRightMotorSpeedDirection(activationFunction(angleR) * w_A + VELOCITY_OFFSET, 1);
//	motor_control->setLeftMotorSpeedDirection(activationFunction(angleL) * w_A + VELOCITY_OFFSET, 1);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activationFunction(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activationFunction(angleR) + VELOCITY_OFFSET) << std::endl;
}
