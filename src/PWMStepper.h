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
    ledc_timer_t   ledcTimer;
    uint8_t        ledcResolutionBits = 8;   // runtime-selected bits
    ledc_clk_cfg_t ledcClk       = LEDC_USE_APB_CLK; // prefer 80 MHz on S3

    StepperState state = STEPPER_OFF;
    StepperMode mode = MODE_LEDC;

    bool stepperEnabledHigh; // true if enable pin is active HIGH
    //bool direction;  // true = forward, false = reverse
    bool invertDirection = false; // true = invert direction logic
    
    // Spinlock for 64-bit variable synchronization between ISR and main thread
    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    
    volatile double currentFreq = 0;
    volatile double encoderFrequency = 0;
    volatile double acceleration;
    volatile double maxFreq;
    volatile double targetFreq;
    volatile int64_t targetPosition;
    volatile int updateNumber = 0;
    volatile double decelerationDistance = 0;
    static const size_t MAX_POSITION_HISTORY = 100; // Max history size
    std::vector<int64_t> positionHistory; // For tracking position over time
    std::vector<uint64_t> updateTimes; // Timestamps of updates

    ESP32Encoder* encoder;
    float encoderScale = 1.0; // Scale factor for encoder counts to steps
    
    // Dual mode operation
    static const double FREQUENCY_THRESHOLD; // 512 Hz - below this we use timer mode
    static const double DECELERATION_FREQUENCY; // 400 Hz - target freq when decelerating
    
    // Timer mode variables
    volatile uint64_t lastSpeedChangeMicros;
    volatile uint32_t stepsSinceLastSpeedChange;
    
    // Deferred command flags for ISR-safe operation
    // ISR sets these flags, main loop processes them via processCommands()
    volatile bool pendingFrequencyChange = false;
    volatile bool pendingStop = false;
    volatile double pendingNewFrequency = 0;
    volatile bool pendingStateChange = false;
    volatile StepperState pendingNewState = STEPPER_OFF;
    
    // Internal ISR-safe methods (only set flags, don't call LEDC)
    void requestStartPWM(double frequency);
    void requestStopPWM();
    
    // Mode selection methods
    void startLEDCMode(double frequency);
    void startTimerMode(double frequency);
    void stopLEDCMode();
    void stopTimerMode();
    void onSpeedChange(double frequency);
    void startPWM(double frequency);
    // Set direction (true = forward, false = reverse)
    void setDirectionPin(bool dir);
    String getStateName(StepperState s) const {
        switch (s) {
            case STEPPER_OFF: return String("OFF");
            case STEPPER_IDLE: return String("IDLE");
            case STEPPER_MOVE_TO_POSITION: return String("MOVE_TO_POSITION");
            case STEPPER_MOVE_WITH_FREQUENCY: return String("MOVE_WITH_FREQUENCY");
            default: return String("UNKNOWN");
        }
    }

    
public:
    // Constructor
    PWMStepper(ESP32Encoder* encoder, float encoderScale, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
    // Destructor for proper cleanup
    ~PWMStepper();
    
    // Initialize the stepper
    void begin();
    
    
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
    void setEncoderScale(float scale) { encoderScale = scale; }
    void setMaxFreq(int64_t maxFrequency) { maxFreq = maxFrequency; }   
    void setStepperEnabledHigh(bool enabled) { stepperEnabledHigh = enabled; }
    void setInvertDirection(bool invert) { invertDirection = invert; }
    
    // Get current status
    bool isEnabled() const;
    bool getDirection() const;
    double getFrequency() const;
    int64_t getTargetPosition() const;
    bool isMoving() const { return state == STEPPER_MOVE_TO_POSITION || state == STEPPER_MOVE_WITH_FREQUENCY; }
    StepperMode getMode() const;
    int64_t getPosition() const {
        return (int) encoder->getCount() * encoderScale;
    }
    void printStatus() const;
    
    // Utility functions
    void update();          // Called from ISR - only sets flags, no LEDC calls
    void processCommands(); // Called from main loop - processes pending LEDC commands
        
    // Advanced functions
    void setLEDCResolution(uint8_t resolution);
    void setLEDCChannel(uint8_t channel);
    
    // Get maximum LEDC frequency for current resolution
    double getMaxLEDCFrequency() const;
};

// Global function to process pending LEDC commands - call from loop()
extern void processAllStepperCommands();

#endif // PWMSTEPPER_H