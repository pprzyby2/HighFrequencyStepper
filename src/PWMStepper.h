#ifndef PWMSTEPPER_H
#define PWMSTEPPER_H

#include <Arduino.h>
#include "ESP32Encoder.h"
#include <vector>
#include "driver/ledc.h"

enum StepperState {
    STEPPER_OFF = 0,
    STEPPER_IDLE = 1,
    STEPPER_MOVE_TO_POSITION = 2,
    STEPPER_MOVE_WITH_FREQUENCY = 3
};

enum StepperMode {
    MODE_LEDC = 0,
    MODE_TIMER = 1
};

class PWMStepper {
private:
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    
    uint8_t ledcChannel;
    uint32_t ledcFrequency = 1000; // Default 1 kHz
    uint8_t ledcResolution = 4;     // Default 4-bit resolution

   // Add explicit LEDC config for ESP32-S3 (LS only)
    ledc_mode_t    ledcSpeedMode = LEDC_LOW_SPEED_MODE;
    ledc_timer_t   ledcTimer     = LEDC_TIMER_0;
    uint8_t        ledcResolutionBits = 8;   // runtime-selected bits
    ledc_clk_cfg_t ledcClk       = LEDC_USE_APB_CLK; // prefer 80 MHz on S3

    StepperState state = STEPPER_OFF;
    StepperMode mode = MODE_LEDC;

    bool stepperEnabledHigh; // true if enable pin is active HIGH
    bool direction;  // true = forward, false = reverse
    bool invertDirection = false; // true = invert direction logic
    volatile double currentFreq = 0;
    volatile double acceleration;
    volatile double maxFreq;
    volatile double targetFreq;
    volatile int64_t targetPosition;
    volatile int updateNumber = 0;
    volatile bool decelerating = false;
    volatile double decelerationDistance = 0;
    static const size_t MAX_POSITION_HISTORY = 100; // Max history size
    std::vector<int64_t> positionHistory; // For tracking position over time
    std::vector<uint64_t> updateTimes; // Timestamps of updates

    ESP32Encoder* encoder;
    float encoderScale = 1.0; // Scale factor for encoder counts to steps
    
    // Dual mode operation
    static const double FREQUENCY_THRESHOLD; // 512 Hz threshold
    
    // Timer mode variables
    volatile uint64_t lastSpeedChangeMicros;
    volatile uint32_t stepsSinceLastSpeedChange;
    
    // Mode selection methods
    void startLEDCMode(double frequency);
    void startTimerMode(double frequency);
    void stopLEDCMode();
    void stopTimerMode();
    void onSpeedChange(double frequency, bool dir);
    void startPWM(double frequency);

    
public:
    // Constructor
    PWMStepper(ESP32Encoder* encoder, float encoderScale, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
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
    void accelerateToFrequency(double frequency);
    void moveAtFrequency(double frequency);
    void moveToPosition(int64_t position, double frequency);
    bool isMovingToPosition() const;
    
    // Stop PWM
    void stopPWM();
    
    // Set frequency while running
    void setTargetFrequency(double frequency);
    void setAcceleration(double accel);
    void setTargetPosition(int64_t position);
    void setMaxFreq(int64_t maxFrequency) { maxFreq = maxFrequency; }   
    void setStepperEnabledHigh(bool enabled) { stepperEnabledHigh = enabled; }
    void setInvertDirection(bool invert) { invertDirection = invert; }
    
    // Get current status
    bool isEnabled() const;
    bool getDirection() const;
    double getFrequency() const;
    int64_t getTargetPosition() const { return targetPosition; }
    bool isMoving() const { return state == STEPPER_MOVE_TO_POSITION || state == STEPPER_MOVE_WITH_FREQUENCY; }
    StepperMode getMode() const;
    int64_t getPosition() const {
        return (int) encoder->getCount() * encoderScale;
    }
    void printStatus() const;
    
    // Utility functions
    void step(uint32_t steps, double frequency, bool dir);
    void moveSteps(int32_t steps, double frequency);
    void update();
        
    // Advanced functions
    void setLEDCResolution(uint8_t resolution);
    void setLEDCChannel(uint8_t channel);
    
    // Get maximum LEDC frequency for current resolution
    double getMaxLEDCFrequency() const;
};

#endif // PWMSTEPPER_H