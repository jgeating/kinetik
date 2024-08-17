#ifndef _Lights_
#define _Lights_

#include <Adafruit_NeoPixel.h>

#define LED_COUNT 28    // # of led's per strip (not total)

class Lights
{
private:
    int strip_count;
    int led_pin;
    Adafruit_NeoPixel *strip;
    //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
    //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
    //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
    //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
    //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

public:
    Lights(int strip_count, int led_pin);

    // Functions
    void setDouble(int i_strip, double input);
    void clear();
    void signal(int i_strip, int location, int r, int g, int b);
};

#endif
