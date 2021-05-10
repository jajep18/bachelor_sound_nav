#include "../includes/navigation.h"

navigation::navigation(MotorControl* motorControl_)
	: motorControl{ motorControl_ }{
	std::cout << "Navigation ready!\n";
}

double navigation::activationFunction(double input) {
	/* Test functions */
	return 20 / (1 + exp(-10 * input)) + 20;	//Sigmoid

}

double navigation::avoidanceActivationFunction(double input){
    return 2 / (1 + exp(-6 * input)) - 1;	//Sigmoid;
}

void navigation::obstacleReflex(double angleToObstacle, double curDis, double prevDis, double prevPrevDis)
{
    //LEFT OR RIGHT REFLEX DODGING
 if (angleToObstacle <= 180){ //RIGHT SIDE OBSTACLE
     double angleNorm = (angleToObstacle - 90) / 90;
     motorControl->setRightMotorSpeedDirection(activationFunction(angleNorm));
     motorControl->setLeftMotorSpeedDirection(activationFunction(-angleNorm));

 }
 else { // angleToObst > 180 //LEFT SIDE OBSTACLE
     double angleNorm = (90 - (angleToObstacle - 180)) / 90;
     motorControl->setRightMotorSpeedDirection(activationFunction(-angleNorm));
     motorControl->setLeftMotorSpeedDirection(activationFunction(angleNorm));
 }
 //Update weight used for vLearning
 wReflexVar = wReflexVar + reflexLearningRate * ((REFLEX_THRESHOLD - curDis) / REFLEX_THRESHOLD) * (prevPrevDis - prevDis) / REFLEX_THRESHOLD;
 reflexCounter += 1;
}

void navigation::obstacleAvoidance(double angleToObstacle, double soundAngle, double curDis, double prevDis, std::ofstream& outputStream)
{
    vLearning = ( (AVOIDANCE_THRESHOLD - curDis)  / (AVOIDANCE_THRESHOLD -REFLEX_THRESHOLD)) * wReflexVar + ((AVOIDANCE_THRESHOLD - prevDis) / (AVOIDANCE_THRESHOLD - REFLEX_THRESHOLD)) * wReflexConst;

    if (angleToObstacle <= 180){  //RIGHT SIDE OBSTACLE
        double angleNorm = (angleToObstacle - 90) / 90;
        double rightMotorOffset = avoidanceActivationFunction(angleNorm) * vLearning;
        double leftMotorOffset = avoidanceActivationFunction(-angleNorm) * vLearning;
        braitenberg(soundAngle, outputStream, leftMotorOffset, rightMotorOffset);
    }
    else { // angleToObst > 180 //LEFT SIDE OBSTACLE
        double angleNorm = (90 - (angleToObstacle - 180)) / 90;
        double rightMotorOffset = avoidanceActivationFunction(-angleNorm) * vLearning;
        double leftMotorOffset = avoidanceActivationFunction(angleNorm) * vLearning;
        braitenberg(soundAngle, outputStream, leftMotorOffset, rightMotorOffset);
    }
}

void navigation::updateState(double distToObstCurrent, double reflexDistToObstCurrent, double soundEnergy, states &CURRENT_STATE_, std::string detectedObj, double objConf, double distToObj, double soundAngle)
{
    if (reflexDistToObstCurrent < REFLEX_THRESHOLD) { //Reflex avoidance
        CURRENT_STATE_ = REFLEX;
    }
    else if (soundEnergy > ENERGY_THRESHOLD) {

        if( detectedObj == "person" && objConf >= 0.70 && distToObj <= 400 && soundAngle <= 190 && soundAngle >= 170) {
            CURRENT_STATE_ = TARGET_FOUND;
        }
        else if (distToObstCurrent < AVOIDANCE_THRESHOLD) { //Obstacle avoidance - obstacle inside avoidance threshold
            CURRENT_STATE_ = AVOID;
        }

        else {	//Braitenberg
            CURRENT_STATE_ = NAVIGATE;
        }
    }
    else { // If no sound
        if (detectedObj == "person" && objConf >= 0.70 && /* distToObj <= 400 && */ soundAngle <= 190 && soundAngle >= 170) { //If person is detected
            CURRENT_STATE_ = NAVIGATE_TO_PERSON;
        }
        else //No sound no person = stop
            CURRENT_STATE_ = WAIT;
    }
}

void  navigation::braitenberg(double angle, std::ofstream& outputStream, double avoidanceLeft, double avoidanceRight)  { //Braitenberg aggression vehicle

	// Update sensor signals
	angleL = (((360 - angle) - 180) / 180); // Normalize
	angleR = (angle - 180) / 180; // Normalize

	//Calculate motorspeed using activation function
	motorSpeedL = activationFunction(angleL) + avoidanceLeft;
	motorSpeedR = activationFunction(angleR) + avoidanceRight + 3;

	//std::cout << "Motorspeed left: " << motorSpeedL << ". Motorspeed right: " << motorSpeedR << std::endl;

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
	//std::cout << "Left speed: " << (activationFunction(angleL) + VELOCITY_OFFSET) << " - Right speed: " << (activationFunction(angleR) + VELOCITY_OFFSET) << std::endl;
}



//This is used for debugging and test purposes
void navigation::manualInputSteering(Vision * vision_, std::ofstream& outputStream){
    std::cout << "Manual steering enabled - Control with WASD - Type 'r' to resume" << std::endl;
    bool runBool = true, printToFile = true;
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
            case 'i': //Shows and prints ICO values
                if(printToFile){
                    std::cout << "\n wReflexVar " << wReflexVar << " vLearning " << vLearning << " reflexCounter " << reflexCounter << std::endl;
                    outputStream << wReflexVar <<  ','<< vLearning << ',' << reflexCounter <<'\n';
                    printToFile = false;
                }
                motorControl->setMatrixVoiceLED(MATRIX_LED_R_9,MAX_BRIGHTNESS,0,MAX_BRIGHTNESS); //Purple
                usleep(1000000);
                motorControl->setMatrixVoiceLED(MATRIX_LED_R_9,0,0,0); //none
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

