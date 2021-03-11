#include "../includes/navigation.h"



navigation::navigation(MotorControl* motorControl_)
	: motorControl{ motorControl_ }{
	std::cout << "Navigation ready!\n";
}

double navigation::activationFunction(double input) {
	return 20 / (1 + exp(-10*input)) + 20;		//Sigmoid v4 or Logistic                       [20,40]
//	return 20 / (1 + exp(-10*input)) + 15;		//Sigmoid v3 or Logistic                       [0,1]
//	return 30 / (1 + exp(-10*input)) + 10;		//Sigmoid v2 or Logistic                       [0,1]
	//return 50 / (1 + exp(-3*input));		//Sigmoid or Logistic                       [0,1]
	//return 30 * tanh(3 * input);			//Hyperbolic tangent (tanh)                 [-1,1]
	//return 30 * atan(5 * input);			//Inverse Hyperbolic Tangent (arctanh)		[-1,1]

	//return 20 * 2 * atan(tanh(5 * input));	//Gudermannian								[-pi/2, pi/2]
}

void navigation::braitenberg(double angle, std::ofstream& outputBraitenberg) { //Braitenberg aggression vehicle

	// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

	//Calculate motorspeed using activation function
	motorSpeedL = activationFunction(angleL) + VELOCITY_OFFSET;
	motorSpeedR = activationFunction(angleR) + VELOCITY_OFFSET;

	//if (angle <= 190 && angle >= 170) //Sound source is front  <---- Test dette, om det er nødvendigt
	//	motorCommand = FORWARD;
	//else if (motorSpeedL > motorSpeedR) //Sound source is right
	//	motorCommand = RIGHTTURN;
	//else if (motorSpeedL < motorSpeedR) //Sound source is left
	//	motorCommand = LEFTTURN;
	//else
	//	motorCommand = STOP;

	std::cout << "Motorspeed left: " << motorSpeedL << ". Motorspeed right: " << motorSpeedR << std::endl;

	//motorControl->changeMotorCommand(motorCommand, motorSpeedL, motorSpeedR);
	motorControl->setLeftMotorSpeedDirection(motorSpeedL, FORWARD);
	motorControl->setRightMotorSpeedDirection(motorSpeedR,FORWARD);

	outputBraitenberg << angleL << "," << motorSpeedL << "," << angleR << "," << motorSpeedR << std::endl;
}


void navigation::navigationICO(double angle, double w_A) {
	//	if (angle < 180) { //Object is on RIGHT side
	//		motor_control->setMatrixVoiceLED(MATRIX_LED_R_1, 0, MAX_BRIGHTNESS, 0);
	//	}
	//	else { // angle >= 180 //object is on LEFT side
	//		motor_control->setMatrixVoiceLED(MATRIX_LED_L_9, 0, MAX_BRIGHTNESS, 0);
	//	}

		// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

//	motor_control->setRightMotorSpeedDirection(activationFunction(angleR) * w_A + VELOCITY_OFFSET, 1);
//	motor_control->setLeftMotorSpeedDirection(activationFunction(angleL) * w_A + VELOCITY_OFFSET, 1);
	//TEST - Print motor values
	std::cout << "Left speed: " << (activationFunction(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activationFunction(angleR) + VELOCITY_OFFSET) << std::endl;
}
