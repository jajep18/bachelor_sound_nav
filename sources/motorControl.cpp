#include "../includes/motorControl.h"

//Constructor
MotorControl::MotorControl(matrix_hal::MatrixIOBus* bus_, matrix_hal::Everloop* everloop_, matrix_hal::EverloopImage* everloop_image_, matrix_hal::GPIOControl* gpio_){
	/*********   INITIALISE MATRIX VOICE DEVICE   **********************/

	bus = bus_;
	everloop = everloop_;
	everloop_image = everloop_image_;
	gpio = gpio_;

	startupShowLEDRainbow();											// Display rainbow animation

	/****************  INITIALISE MOTOR CONTROL   **********************/

	initGPIOPins(/*&gpio*/);											// Initialise Matrix Voice GPIO pins
}

//Deconstructor
MotorControl::~MotorControl()
{
}



// Initialise Matrix Voice GPIO pins
void MotorControl::initGPIOPins()
{

	gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA, 1);							//Pin mode as output
	gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA, 1);						// Pin function as PWM
	gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1, 1);
	gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2, 1);

	gpio->SetMode(TB6612_LEFT_MOTOR_PWMB, 1);							//Pin mode as output
	gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB, 1);						// Pin function as PWM
	gpio->SetMode(TB6612_LEFT_MOTOR_BIN1, 1);
	gpio->SetMode(TB6612_LEFT_MOTOR_BIN2, 1);
}

// Set speed and direction of LEFT motor
// Directiom -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void MotorControl::setLeftMotorSpeedDirection(/*matrix_hal::GPIOControl* gpio,*/ double speed, int dir)
{
	if (speed <= 0 ) // Reverse
	{
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 0); // Rotate left motor clockwise
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 1);
	}
	if ((speed > 0) || (speed >= 1)) // Forward
	{
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN1, 1); // Rotate left motor counter-clockwise
		gpio->SetGPIOValue(TB6612_LEFT_MOTOR_BIN2, 0);
	}

	// Set motor speed via PWM signal (min. = -100, max. = 100)
	if (speed > 100)
		speed = 100;
	else if (speed < -100)
		speed = -100;

	gpio->SetPWM(1000, abs(speed), TB6612_LEFT_MOTOR_PWMB);
}

// Set speed and direction of RIGHT motor
// Direction -> 1 = forward, 0 = reverse
// Speed -> 0-100% in steps of 1%
void MotorControl::setRightMotorSpeedDirection(/*matrix_hal::GPIOControl* gpio,*/ double speed, int dir)
{

	if (speed <= 0 || dir <= 0) // Reverse
	{
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 0); // Rotate right motor counter-clockwise
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 1);
	}
	if (speed >= 1 || dir > 0) // Forward
	{
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN1, 1); // Rotate right motor clockwise
		gpio->SetGPIOValue(TB6612_RIGHT_MOTOR_AIN2, 0);
	}

	// Set motor speed via PWM signal (min. = -100, max. = 100)
	if (speed > 100)
		speed = 100;
	if (speed < -100)
		speed = -100;

	gpio->SetPWM(1000, abs(speed), TB6612_RIGHT_MOTOR_PWMA);
}

void MotorControl::changeMotorCommand(int commandInput, double speedL, double speedR)
{
	switch (commandInput)
	{
	case STOP:
		setLeftMotorSpeedDirection(STOP, FORWARD );
		setRightMotorSpeedDirection(STOP, FORWARD);
		break;

	case FORWARD:
		setLeftMotorSpeedDirection(speedL, FORWARD);
		setRightMotorSpeedDirection(speedR, FORWARD);
		break;

	case REVERSE:
		setLeftMotorSpeedDirection(REVERSE_SPEED, REVERSE);
		setRightMotorSpeedDirection(REVERSE_SPEED, REVERSE);
		break;

	case LEFT_TURN:
		setLeftMotorSpeedDirection(speedL, REVERSE);
		setRightMotorSpeedDirection(speedR, FORWARD);
		break;

	case RIGHT_TURN:
		setLeftMotorSpeedDirection(speedL, FORWARD);
		setRightMotorSpeedDirection(speedR, REVERSE);
		break;

	default:
		std::cout << "Invalid motor command!\n";
		break;
	}

}



// Display a rainbow animation on the Matrix Voice LEDs
void MotorControl::startupShowLEDRainbow(/*matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image*/)
{
	// Variables used for sine wave rainbow logic
	float counter = 0;
	const float freq = 0.375;

	// For each led in everloop_image.leds, set led value
	for (matrix_hal::LedValue& led : everloop_image->leds)
	{
		// Sine waves 120 degrees out of phase for rainbow
		led.red = (std::sin(freq * counter + (M_PI / 180 * 240)) * 155 + 100) / 10;
		led.green = (std::sin(freq * counter + (M_PI / 180 * 120)) * 155 + 100) / 10;
		led.blue = (std::sin(freq * counter + 0) * 155 + 100) / 10;
		counter = counter + 1.01;
		// Updates the LEDs
		everloop->Write(everloop_image);
		usleep(40000);
	}

	usleep(460000);

	// For each led in everloop_image.leds, set led value to 0
	for (matrix_hal::LedValue& led : everloop_image->leds)
	{
		// Turn off Everloop
		led.red = 0;
		led.green = 0;
		led.blue = 0;
		led.white = 0;
	}

	// Updates the Everloop on the MATRIX device
	everloop->Write(everloop_image);
}

// Set individual LEDs on the Matrix Voice
void MotorControl::setMatrixVoiceLED( int ledn, int r, int g, int b)
{
	if (ledn < 18)
	{
		everloop_image->leds[ledn].red = r;
		everloop_image->leds[ledn].green = g;
		everloop_image->leds[ledn].blue = b;
		everloop_image->leds[ledn].white = 0;
	}
	everloop->Write(everloop_image);
}

//Turn off all matrix voice LEDS
void MotorControl::resetMatrixVoiceLEDs() {
	for (int i = 0; i < 18; i++) {
		setMatrixVoiceLED(i, 0, 0, 0);
	}
}

