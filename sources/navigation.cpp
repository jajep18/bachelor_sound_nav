#include "../includes/navigation.h"

navigation::navigation(MotorControl* motorControl_)
	: motorControl{ motorControl_ }{
	std::cout << "Navigation ready!\n";
}

double navigation::activationFunction(double input) {
	/* Test functions */
	return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid

}

void navigation::obstacleReflex(double angleToObstacle, double curDis, double prevDis, double prevPrevDis)
{
    //LEFT OR RIGHT REFLEX DODGING
 if (angleToObstacle <= 180){ //RIGHT SIDE OBSTACLE
     double angleNorm = (angleToObstacle - 90) / 90;
     motorControl.setRightMotorSpeedDirection(activationFunction(angleNorm));
     motorControl.setLeftMotorSpeedDirection(activationFunction(-angleNorm));

 }
 else { // angleToObst > 180 //LEFT SIDE OBSTACLE
     double angleNorm = (90 - (angleToObstacle - 180)) / 90;
     motorControl.setRightMotorSpeedDirection(activationFunction(-angleNorm));
     motorControl.setLeftMotorSpeedDirection(activationFunction(angleNorm));
 }
 //Update weight used for vLearning
 wReflexVar = wReflexVar + reflexLearningRate * (curDis / REFLEX_THRESHOLD) * (prevDis - prevPrevDis) / REFLEX_THRESHOLD;
 reflexCounter += 1;
}

void navigation::obstacleAvoidance(double angleToObstacle, double curDis, double prevDis)
{
    vLearning = (curDis / REFLEX_THRESHOLD) * wReflexVar + (prevDis / REFLEX_THRESHOLD) * wReflexConst;

    if (angleToObst <= 180){  //RIGHT SIDE OBSTACLE
        navigation.braitenberg(soundLocalization.getAngle(), outputStream, 0, vLearning);
    }
    else { // angleToObst > 180 //LEFT SIDE OBSTACLE
        navigation.braitenberg(soundLocalization.getAngle(), outputStream, vLearning, 0);
    }
}

void  navigation::braitenberg(double angle, std::ofstream& outputStream, double avoidanceLeft, double avoidanceRight)  { //Braitenberg aggression vehicle

	// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

	//Calculate motorspeed using activation function
	motorSpeedL = activationFunction(angleL) + avoidanceLeft;
	motorSpeedR = activationFunction(angleR) + avoidanceRight;

	std::cout << "Motorspeed left: " << motorSpeedL << ". Motorspeed right: " << motorSpeedR << std::endl;

	//motorControl->changeMotorCommand(motorCommand, motorSpeedL, motorSpeedR);
	motorControl->setLeftMotorSpeedDirection(motorSpeedL, FORWARD);
	motorControl->setRightMotorSpeedDirection(motorSpeedR,FORWARD);

	outputStream << angleL << "," << motorSpeedL << "," << angleR << "," << motorSpeedR << std::endl;
}

void navigation::navigationICO(double angle, double w_A) {

		// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize


	//TEST - Print motor values
	std::cout << "Left speed: " << (activationFunction(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activationFunction(angleR) + VELOCITY_OFFSET) << std::endl;
}

//This is used for debugging and test purposes
void navigation::manualInputSteering(Vision * vision_){
    std::cout << "Manual steering enabled - Control with WASD - Type 'r' to resume" << std::endl;
    bool runBool = true;
    while(runBool){
        switch(vision_->inputKey){
            case 'w':
                motorControl->changeMotorCommand(FORWARD, 25, 25);
                break;
            case 's':
                motorControl->changeMotorCommand(REVERSE, 25, 25);
                break;
            case 'a':
                motorControl->changeMotorCommand(LEFT_TURN, 18, 25);
                break;
            case 'd':
                motorControl->changeMotorCommand(RIGHT_TURN, 25, 18);
                break;
            case 27: //27 = 'ESC'
            case 'r':
                runBool = false;
                break;
            default:
                motorControl->changeMotorCommand(STOP, 0 , 0);
                break;
        }
        usleep(50000);

    }
    std::cout << "Manual steering disabled - control loop resumes" << std::endl;
}

