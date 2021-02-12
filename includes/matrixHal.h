#pragma once
class matrixHal
{
	void initGPIOPins(matrix_hal::GPIOControl* gpio) // Initialise Matrix Voice GPIO pins
	void startupShowLEDRainbow(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image) //Does what it says
	void setMatrixVoiceLED(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image, int ledn, int r, int g, int b) // Set individual LEDs on the Matrix Voice
}; 

