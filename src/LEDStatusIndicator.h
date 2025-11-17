#ifndef LEDSTATUSINDICATOR_H
#define LEDSTATUSINDICATOR_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "HighFrequencyStepper.h"

// LED Status States
enum LEDStatus {
    LED_IDLE,           // Green - System idle, no motors running
    LED_MOVING,         // Blue - Motor(s) actively moving
    LED_ERROR,          // Red - Error detected
    LED_STALL,          // Yellow - Stall detected
    LED_INITIALIZING,   // Cyan - System initializing
    LED_DISABLED,       // Orange - Motor(s) disabled
    LED_WARNING,        // Magenta - Warning condition
    LED_OFF             // Black - LED off
};

class LEDStatusIndicator {
private:
    Adafruit_NeoPixel* pixel;
    uint8_t pin;
    uint8_t brightness;
    LEDStatus currentStatus;
    unsigned long lastBlinkTime;
    bool blinkState;
    uint16_t blinkInterval;
    
    // Color definitions (RGB)
    struct Color {
        uint8_t r, g, b;
        
        Color(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) 
            : r(red), g(green), b(blue) {}
    };
    
    // Status color mapping
    Color getColorForStatus(LEDStatus status);
    
    // Apply color to LED
    void setColor(Color color);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    
public:
    // Constructor
    LEDStatusIndicator(uint8_t ledPin = 48, uint8_t ledBrightness = 50);
    
    // Destructor
    ~LEDStatusIndicator();
    
    // Initialization
    bool begin();
    
    // Status control
    void setStatus(LEDStatus status);
    LEDStatus getStatus() const { return currentStatus; }
    
    // Automatic status update from stepper controller
    void updateFromStepper(HighFrequencyStepper& controller, uint8_t stepperIndex);
    void updateFromAllSteppers(HighFrequencyStepper& controller);
    
    // Brightness control
    void setBrightness(uint8_t brightness);
    uint8_t getBrightness() const { return brightness; }
    
    // Blinking control
    void enableBlink(uint16_t intervalMs = 500);
    void disableBlink();
    void update(); // Call in loop() for blink effect
    
    // Direct color control
    void setRGB(uint8_t r, uint8_t g, uint8_t b);
    void setRainbow(uint8_t wheelPos);
    
    // Utility
    void turnOff();
    void test(); // Cycle through all colors
};

#endif // LEDSTATUSINDICATOR_H
