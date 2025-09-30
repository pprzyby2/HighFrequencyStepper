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
    double currentFreq;
    
    // Dual mode operation
    static const double FREQUENCY_THRESHOLD; // 512 Hz threshold
    bool isLEDCMode;
    
    // Timer mode variables
    hw_timer_t* stepTimer;
    volatile bool timerStepState;
    volatile uint32_t timerStepsRemaining;
    volatile bool timerRunning;
    
    // Timer callback function
    static void IRAM_ATTR onStepTimer();
    void IRAM_ATTR handleStepTimer();
    
    // Mode selection methods
    void startLEDCMode(double frequency);
    void startTimerMode(double frequency);
    void stopLEDCMode();
    void stopTimerMode();
    
public:
    // Constructor
    PWMStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
    // Destructor for proper cleanup
    ~PWMStepper();
    
    // Initialize the stepper
    void begin();
    
    // Set direction (true = forward, false = reverse)
    void setDirection(bool dir);
    
    // Enable/disable the stepper driver
    void enable();
    void disable();
    
    // Start PWM at specified frequency (steps per second)
    void startPWM(double frequency);
    
    // Stop PWM
    void stopPWM();
    
    // Set frequency while running
    void setFrequency(double frequency);
    
    // Get current status
    bool isEnabled() const;
    bool getDirection() const;
    double getFrequency() const;
    bool isInLEDCMode() const;
    
    // Utility functions
    void step(uint32_t steps, double frequency, bool dir);
    void moveSteps(int32_t steps, double frequency);
    
    // Timer mode specific functions
    void stepTimerMode(uint32_t steps, double frequency, bool dir);
    bool isTimerStepComplete() const;
    
    // Advanced functions
    void setLEDCResolution(uint8_t resolution);
    void setLEDCChannel(uint8_t channel);
};

#endif // PWMSTEPPER_H