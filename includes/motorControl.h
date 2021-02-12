#pragma once
class motorControl
{
private:
	int leftMotorSpeed;
	int rightMotorSpeed;

public:

	void setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir); // Directiom -> 1 = forward, 0 = reverse
	void setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir); // Directiom -> 1 = forward, 0 = reverse

	
};

