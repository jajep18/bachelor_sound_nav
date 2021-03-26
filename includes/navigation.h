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

#include <math.h>
#include <fstream>

class navigation
{
private:

	double angleL = 0, angleR = 0;
	double motorSpeedL = 0, motorSpeedR = 0;
	int motorCommand = 1;

	MotorControl* motorControl;

	//Obstacle avoidance / ICO Learning

	double wReflexVar = 1.0;		// Standard weight that needs to be multiplied with distance to current Obstacle
	double wReflexConst = 1.0;		//
	double reflexLearningRate = 10;	// Learning rate for reflex µ
	double vLearning = 0.0; 		// Velocity to add to the initial velocity
	int reflexCounter = 0;



public:

    navigation(MotorControl* motorControl_);

	void braitenberg(double angle, std::ofstream& outputStream, double avoidanceLeft = 0, double avoidanceRight = 0) ;				//Braitenberg aggression vehicle

	void navigationICO(double angle, double w_A);	//ICO learning - work in progress

	void manualInputSteering(Vision * vision_);

	double activationFunction(double input);

	void obstacleReflex(double angleToObstacle, double curDis, double prevDis, double prevPrevDis);

	void obstacleAvoidance(double angleToObstacle, double curDis, double prevDis);


};

