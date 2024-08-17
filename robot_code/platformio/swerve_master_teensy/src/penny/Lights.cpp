#include "penny/Lights.h"

Lights::Lights(int strip_count, int led_pin)
{
    strip = new Adafruit_NeoPixel(strip_count * LED_COUNT, led_pin, NEO_GRB);
    strip->begin();          // INITIALIZE NeoPixel strip object (REQUIRED)
    strip->show();           // Turn OFF all pixels ASAP
    strip->setBrightness(255); // (max = 255)
    strip->clear();
}

void Lights::setDouble(int i_strip, double val)
{
    double index_double = (val + double(i_strip)) * double(LED_COUNT);
    double remnant = fmod(index_double, 1.0);
    int lowVal = round(remnant * 255.0);
    int index = floor(index_double);
    strip->setPixelColor(index, strip->Color(255 - lowVal, 255 - lowVal, 255 - lowVal));
    strip->setPixelColor(index + 1, strip->Color(lowVal, lowVal, lowVal));
    strip->show();
}

void Lights::signal(int i_strip, int location, int r, int g, int b)
{
    int start;
    int end;
    if (location == 0)
    {
        start = 0;
        end = 3;
    }
    else
    {
        start = LED_COUNT - 4;
        end = LED_COUNT - 1;
    }
    for (int i = start; i < end; i++)
    {
        strip->setPixelColor(i + i_strip * LED_COUNT, strip->Color(r, g, b));
    }
    strip->show();
}

void Lights::clear()
{
    strip->clear();
}
