#ifndef PWMSTEPPER_H
#define PWMSTEPPER_H

/**
 * @file PWMStepper.h
 * @brief Low-level stepper motor driver using ESP32 LEDC PWM and timer-based stepping.
 * 
 * This class provides precise step pulse generation for stepper motors using dual-mode
 * operation: LEDC hardware PWM for high frequencies (>=512 Hz) and software timer-based
 * stepping for low frequencies (<512 Hz). It includes position tracking via encoder
 * feedback and acceleration/deceleration control.
 * 
 * @note This class uses an ISR-safe architecture where the update() function runs in
 * an ISR context and sets flags, while processCommands() runs in a separate FreeRTOS
 * task to handle LEDC configuration changes.
 * 
 * @author ESP32-STEPPER2 Project
 */

#include <Arduino.h>
#include "ESP32Encoder.h"
#include <vector>
#include "driver/ledc.h"

/**
 * @brief Stepper motor operational states
 */
enum StepperState {
    STEPPER_OFF = 0,              ///< Driver disabled, no movement
    STEPPER_IDLE = 1,             ///< Driver enabled, not moving
    STEPPER_MOVE_TO_POSITION = 2, ///< Moving to target position with acceleration
    STEPPER_MOVE_WITH_FREQUENCY = 3 ///< Continuous movement at target frequency
};

/**
 * @brief PWM generation mode
 */
enum StepperMode {
    MODE_LEDC = 0,  ///< Hardware LEDC PWM (for frequencies >= 512 Hz)
    MODE_TIMER = 1  ///< Software timer-based stepping (for frequencies < 512 Hz)
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
    volatile int reachedTargetCounter = 0; // Counts how many consecutive updates we've been at the target (for stability)
    static const size_t MAX_POSITION_HISTORY = 100; // Max history size
    std::vector<int64_t> positionHistory; // For tracking position over time
    std::vector<uint64_t> updateTimes; // Timestamps of updates

    ESP32Encoder* encoder;
    float encoderScale = 1.0; // Scale factor for encoder counts to steps
    
    // Dual mode operation
    static const double FREQUENCY_THRESHOLD; // 1024 Hz - below this we use timer mode
    static const double DECELERATION_FREQUENCY; // 1000 Hz - target freq when decelerating
    
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
    /**
     * @brief Construct a new PWMStepper object
     * @param encoder Pointer to ESP32Encoder for position feedback
     * @param encoderScale Ratio of encoder counts to microsteps
     * @param stepPin GPIO pin for step pulses
     * @param dirPin GPIO pin for direction control
     * @param enablePin GPIO pin for driver enable
     * @param ledcChannel LEDC channel (0-15), each stepper needs unique channel
     * @note Maximum 4 steppers supported (limited by LEDC timers)
     */
    PWMStepper(ESP32Encoder* encoder, float encoderScale, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel = 0);
    
    /**
     * @brief Destructor - stops PWM and releases resources
     */
    ~PWMStepper();
    
    /**
     * @brief Initialize the stepper driver
     * @note Must be called before any movement commands. Configures GPIO pins,
     *       LEDC channel, and starts the command processing task.
     */
    void begin();
    
    
    /**
     * @brief Enable the stepper driver (energize coils)
     */
    void enable();
    
    /**
     * @brief Disable the stepper driver (de-energize coils)
     * @note Also stops any ongoing movement
     */
    void disable();
    
    /**
     * @brief Accelerate to target frequency with configured acceleration
     * @param frequency Target frequency in Hz (steps per second), negative for reverse
     * @note Non-blocking, motor accelerates gradually to target frequency
     */
    void accelerateToFrequency(double frequency);
    
    /**
     * @brief Start moving immediately at specified frequency (no acceleration)
     * @param frequency Target frequency in Hz (steps per second), negative for reverse
     * @note Immediate speed change, use accelerateToFrequency() for smooth transitions
     */
    void moveAtFrequency(double frequency);
    
    /**
     * @brief Move to absolute position with trapezoidal motion profile
     * @param position Target position in microsteps
     * @param frequency Maximum frequency in Hz for the move
     * @note Uses acceleration/deceleration for smooth motion
     */
    void moveToPosition(int64_t position, double frequency);
    
    /**
     * @brief Check if currently executing a moveToPosition command
     * @return true if moving to position, false otherwise
     */
    bool isMovingToPosition() const;
    
    /**
     * @brief Stop PWM output immediately
     * @note Sets frequency to 0 and returns to IDLE state
     */
    void stopPWM();
    
    /**
     * @brief Set target frequency (thread-safe)
     * @param frequency Target frequency in Hz
     */
    void setTargetFrequency(double frequency);
    
    /**
     * @brief Set acceleration rate (thread-safe)
     * @param accel Acceleration in steps/s²
     */
    void setAcceleration(double accel);
    
    /**
     * @brief Set target position (thread-safe)
     * @param position Target position in microsteps
     */
    void setTargetPosition(int64_t position);
    
    /**
     * @brief Set encoder scale factor
     * @param scale Ratio of encoder counts to microsteps
     */
    void setEncoderScale(float scale) { encoderScale = scale; }
    
    /**
     * @brief Set maximum allowed frequency
     * @param maxFrequency Maximum frequency in Hz
     */
    void setMaxFreq(int64_t maxFrequency) { maxFreq = maxFrequency; }
    
    /**
     * @brief Configure enable pin polarity
     * @param enabled true if enable pin is active HIGH, false for active LOW
     */
    void setStepperEnabledHigh(bool enabled) { stepperEnabledHigh = enabled; }
    
    /**
     * @brief Invert direction logic
     * @param invert true to invert direction, false for normal
     */
    void setInvertDirection(bool invert) { invertDirection = invert; }
    
    /**
     * @brief Check if stepper driver is enabled
     * @return true if enabled, false otherwise
     */
    bool isEnabled() const;
    
    /**
     * @brief Get current movement direction
     * @return true for forward, false for reverse
     */
    bool getDirection() const;
    
    /**
     * @brief Get current step frequency (thread-safe)
     * @return Current frequency in Hz
     */
    double getFrequency() const;
    
    /**
     * @brief Get target position (thread-safe)
     * @return Target position in microsteps
     */
    int64_t getTargetPosition() const;
    
    /**
     * @brief Check if motor is currently moving
     * @return true if moving (to position or at frequency)
     */
    bool isMoving() const { return state == STEPPER_MOVE_TO_POSITION || state == STEPPER_MOVE_WITH_FREQUENCY; }
    
    /**
     * @brief Get current PWM generation mode
     * @return MODE_LEDC or MODE_TIMER
     */
    StepperMode getMode() const;
    
    /**
     * @brief Get current position from encoder
     * @return Current position in microsteps
     */
    int64_t getPosition() const {
        return (int) encoder->getCount() * encoderScale;
    }
    
    /**
     * @brief Print current status to Serial
     */
    void printStatus() const;
    
    /**
     * @brief ISR update function - called from timer ISR
     * @note Only sets flags, does not call LEDC functions (ISR-safe)
     * @warning Do not call from user code
     */
    void update();
    
    /**
     * @brief Process pending LEDC commands
     * @note Called automatically by FreeRTOS task, no need to call manually
     */
    void processCommands();
        
    /**
     * @brief Set LEDC resolution
     * @param resolution Resolution in bits (1-16)
     */
    void setLEDCResolution(uint8_t resolution);
    
    /**
     * @brief Set LEDC channel
     * @param channel Channel number (0-15)
     */
    void setLEDCChannel(uint8_t channel);
    
    /**
     * @brief Get maximum achievable LEDC frequency
     * @return Maximum frequency in Hz for current resolution
     */
    double getMaxLEDCFrequency() const;
};

/**
 * @brief Process pending LEDC commands for all stepper instances
 * @note Called automatically by internal FreeRTOS task. Not needed in user loop().
 */
extern void processAllStepperCommands();

#endif // PWMSTEPPER_H