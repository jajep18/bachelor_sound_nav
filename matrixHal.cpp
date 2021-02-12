#include "includes/matrixHal.h"

// Initialise Matrix Voice GPIO pins
void initGPIOPins(matrix_hal::GPIOControl* gpio)
{

    gpio->SetMode(TB6612_RIGHT_MOTOR_PWMA, 1); //Pin mode as output
    gpio->SetFunction(TB6612_RIGHT_MOTOR_PWMA, 1); // Pin function as PWM
    gpio->SetMode(TB6612_RIGHT_MOTOR_AIN1, 1);
    gpio->SetMode(TB6612_RIGHT_MOTOR_AIN2, 1);

    gpio->SetMode(TB6612_LEFT_MOTOR_PWMB, 1); //Pin mode as output
    gpio->SetFunction(TB6612_LEFT_MOTOR_PWMB, 1); // Pin function as PWM
    gpio->SetMode(TB6612_LEFT_MOTOR_BIN1, 1);
    gpio->SetMode(TB6612_LEFT_MOTOR_BIN2, 1);
}



void startupShowLEDRainbow(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image)
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
void setMatrixVoiceLED(matrix_hal::Everloop* everloop, matrix_hal::EverloopImage* everloop_image, int ledn, int r, int g, int b)
{
    for (int i = 0; i < 18; i++)
    {
        if (i == ledn)
        {
            everloop_image->leds[ledn].red = r;
            everloop_image->leds[ledn].green = g;
            everloop_image->leds[ledn].blue = b;
            everloop_image->leds[ledn].white = 0;
        }
    }
    everloop->Write(everloop_image);
}

