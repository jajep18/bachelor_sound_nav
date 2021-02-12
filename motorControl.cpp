#include "includes/motorControl.h"
#include "includes/defines.h"

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void motorControl::setLeftMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
    leftMotorSpeed = speed;
    if (dir <= 0) // Reverse
    {
        gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 0); // Rotate left motor clockwise
        gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 1);
    }
    if ((dir > 0) || (dir >= 1)) // Forward
    {
        gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 1); // Rotate left motor counter-clockwise
        gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 0);
    }

    // Set motor speed via PWM signal (min. = 0, max. = 100)
    if (speed > 100)
        speed = 100;
    if (speed < 0)
        speed = 0;

    gpio->SetPWM(1000, speed, TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void motorControl::setRightMotorSpeedDirection(matrix_hal::GPIOControl* gpio, int speed, int dir)
{
    rightMotorSpeed = speed;
    if (dir <= 0) // Reverse
    {
        gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 0); // Rotate right motor counter-clockwise
        gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 1);
    }
    if ((dir > 0) || (dir >= 1)) // Forward
    {
        gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 1); // Rotate right motor clockwise
        gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 0);
    }

    // Set motor speed via PWM signal (min. = 0, max. = 100)
    if (speed > 100)
        speed = 100;
    if (speed < 0)
        speed = 0;

    gpio->SetPWM(1000, speed, TB6612_RIGHT_MOTOR_PWMA);
}
