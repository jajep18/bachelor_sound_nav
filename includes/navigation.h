#pragma once
/*
 * Description:     ODAS class - implementation of ODAS for MATRIX VOICE.
 *							   - Listens to odas live (has to be run from same terminal before this class is instantiated)
 *
 * Author:			Jacob Fløe Jeppesen
 * Institute:		University of Southern Denmark
 * Creation date:   06-03-2021
 */
#include "motorControl.h"
#include "camera.h"
#include "defines.h"

#include <math.h>
#include <fstream>

enum states { WAIT = 0, NAVIGATE = 1, AVOID = 2, REFLEX = 3, TARGET_FOUND = 4, NAVIGATE_TO_PERSON = 5 };

class navigation
{
private:

	double angleL = 0, angleR = 0;
	double motorSpeedL = 0, motorSpeedR = 0;
	int motorCommand = 1;

	MotorControl* motorControl;

	//Obstacle avoidance / ICO Learning

	double wReflexVar = 7.01413;//6.77988;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double wReflexConst = 5.0;		//
	double reflexLearningRate = 5;	// Learning rate for reflex µ.  0.7 den 14-05 - test1
	double vLearning = 0.0; 		// Velocity to add to the initial velocity
	int reflexCounter = 23;



public:

    navigation(MotorControl* motorControl_);

	void braitenberg(double angle, std::ofstream& outputStream, double avoidanceLeft = 0, double avoidanceRight = 0) ;				//Braitenberg aggression vehicle

	void navigationICO(double angle, double w_A);	//ICO learning - work in progress

	void manualInputSteering(Vision * vision_, std::ofstream& outputStream);

	double activationFunction(double input);
	double avoidanceActivationFunction(double input);

	void obstacleReflex(double angleToObstacle, double curDis, double prevDis, double prevPrevDis);

	void obstacleAvoidance(double angleToObstacle, double soundAngle, double curDis, double prevDis, std::ofstream& outputStream);

	void updateState(double distToObstCurrent, double reflexDistToObstCurrent, double soundEnergy, states& CURRENT_STATE,  std::string detectedObj, double objConf, double distToObj, double soundAngle);





};

