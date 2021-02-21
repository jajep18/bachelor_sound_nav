#pragma once
class motorControl
{
private:
	#define REVERSE                 -1
	#define FORWARD                  1
	#define LEFTTURN                 2
	#define RIGHTTURN                3


	int leftMotorSpeed;
	int rightMotorSpeed;

public:

	void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir); // Directiom -> 1 = forward, 0 = reverse
	void setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir); // Directiom -> 1 = forward, 0 = reverse

	
};

