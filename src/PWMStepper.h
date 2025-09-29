#ifndef PWMSTEPPER_H
#define PWMSTEPPER_H

#include <Arduino.h>

class PWMStepper {
private:
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    
    uint8_t ledcChannel;
    uint32_t ledcFrequency;
    uint8_t ledcResolution;
    
    bool isRunning;
    bool direction;  // true = forward, false = reverse
    uint32_t currentFreq;
    
public:
    // Constructor
    PWMStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
    // Initialize the stepper
    void begin();
    
    // Set direction (true = forward, false = reverse)
    void setDirection(bool dir);
    
    // Enable/disable the stepper driver
    void enable();
    void disable();
    
    // Start PWM at specified frequency (steps per second)
    void startPWM(uint32_t frequency);
    
    // Stop PWM
    void stopPWM();
    
    // Set frequency while running
    void setFrequency(uint32_t frequency);
    
    // Get current status
    bool isEnabled() const;
    bool getDirection() const;
    uint32_t getFrequency() const;
    
    // Utility functions
    void step(uint32_t steps, uint32_t frequency, bool dir);
    void moveSteps(int32_t steps, uint32_t frequency);
    
    // Advanced functions
    void setLEDCResolution(uint8_t resolution);
    void setLEDCChannel(uint8_t channel);
};

#endif // PWMSTEPPER_H