#ifndef PWMSTEPPER_H
#define PWMSTEPPER_H

#include <Arduino.h>
#include "ESP32Encoder.h"
#include <vector>

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
    double acceleration;
    double targetFreq;
    int64_t targetPosition;
    int updateNumber = 0;
    static const size_t MAX_POSITION_HISTORY = 100; // Max history size
    std::vector<int64_t> positionHistory; // For tracking position over time
    std::vector<uint64_t> updateTimes; // Timestamps of updates

    ESP32Encoder* encoder;
    
    // Dual mode operation
    static const double FREQUENCY_THRESHOLD; // 512 Hz threshold
    bool isLEDCMode;
    
    // Timer mode variables
    hw_timer_t* stepTimer;
    volatile bool timerStepState;
    volatile uint32_t timerStepsRemaining;
    volatile bool timerRunning;
    volatile int interruptCount;
    volatile double recentFreq = 0.0;
    volatile int state = 0; // 0 = idle, 1 = accelerating, 2 = cruising, 3 = decelerating
    volatile uint64_t lastSpeedChangeMicros;
    volatile uint32_t stepsSinceLastSpeedChange;
    
    // Timer callback function
    static void IRAM_ATTR onStepTimer();
    void IRAM_ATTR handleStepTimer();
    
    // Mode selection methods
    void startLEDCMode(double frequency);
    void startTimerMode(double frequency);
    void stopLEDCMode();
    void stopTimerMode();
    void onSpeedChange();
    
public:
    // Constructor
    PWMStepper(ESP32Encoder* encoder, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
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
    void setTargetFrequency(double frequency);
    void setFrequency(double frequency);
    void setAcceleration(double accel);
    
    // Get current status
    bool isEnabled() const;
    bool getDirection() const;
    double getFrequency() const;
    bool isInLEDCMode() const;
    int getInterruptCount() const { return interruptCount; } // For debugging
    
    // Utility functions
    void step(uint32_t steps, double frequency, bool dir);
    void moveSteps(int32_t steps, double frequency);
    void update();
    
    // Timer mode specific functions
    void stepTimerMode(uint32_t steps, double frequency, bool dir);
    bool isTimerStepComplete() const;
    
    // Advanced functions
    void setLEDCResolution(uint8_t resolution);
    void setLEDCChannel(uint8_t channel);
    
    // Get maximum LEDC frequency for current resolution
    double getMaxLEDCFrequency() const;
};

#endif // PWMSTEPPER_H