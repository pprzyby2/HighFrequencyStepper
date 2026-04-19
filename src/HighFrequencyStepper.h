#ifndef HIGHFREQUENCYSTEPPER_H
#define HIGHFREQUENCYSTEPPER_H

/**
 * @file HighFrequencyStepper.h
 * @brief High-level multi-stepper controller with TMC driver integration.
 * 
 * This class provides a unified interface for controlling up to 4 stepper motors
 * with support for:
 * - TMC2209 (UART) and TMC2240 (SPI) driver configuration
 * - Encoder-based position feedback
 * - Position and velocity control modes
 * - Automatic acceleration/deceleration profiles
 * - StallGuard detection (TMC drivers)
 * 
 * Architecture:
 * - Uses PWMStepper for low-level step generation
 * - ESP32Encoder for position feedback
 * - TMCStepper library for driver communication
 * 
 * @example
 * ```cpp
 * HighFrequencyStepper controller;
 * StepperConfig config;
 * config.stepPin = 10;
 * config.dirPin = 11;
 * config.enablePin = 12;
 * config.stepsPerRev = 200;
 * config.microsteps = 16;
 * // ... configure other fields
 * controller.addStepper(0, config);
 * controller.initializeStepper(0);
 * controller.enableStepper(0);
 * controller.moveToPosition(0, 1000, 5000, true); // Move to position 1000 at 5kHz
 * ```
 * 
 * @author ESP32-STEPPER2 Project
 */

#include <Arduino.h>
#include "PWMStepper.h"
#include "TMCStepper.h"
#include <ESP32Encoder.h>

/// Maximum number of steppers supported (limited by LEDC timers)
#define MAX_STEPPERS 4

/**
 * @brief TMC driver type selection
 */
enum DriverType {
    STEP_DIR_ONLY = 0,   ///< Basic step/direction driver (no communication)
    TMC2209_DRIVER = 1,  ///< TMC2209 with UART interface
#ifdef TMC2240
    TMC2240_DRIVER = 2   ///< TMC2240 with SPI interface
#endif
};

/**
 * @brief UART configuration for TMC2209
 */
struct UARTConfig {
    HardwareSerial* uart;    ///< Pointer to HardwareSerial instance (Serial1, Serial2)
    uint8_t driverAddress;   ///< Driver address (0-3, set by MS1/MS2 pins)
    float rSense;            ///< Current sense resistor value in Ohms (typically 0.11)
};

#ifdef TMC2240
/**
 * @brief SPI configuration for TMC2240
 */
struct SPIConfig {
    uint8_t pinCS;      ///< Chip Select pin
    uint8_t pinMOSI;    ///< MOSI pin
    uint8_t pinMISO;    ///< MISO pin
    uint8_t pinSCK;     ///< SCK pin
    uint8_t link_index; ///< Link index in SPI daisy chain
};
#endif

/**
 * @brief Combined driver settings
 */
struct StepperDriverSettings {
    DriverType driverType;   ///< Type of driver used
    UARTConfig uartConfig;   ///< UART config (for TMC2209)
#ifdef TMC2240
    SPIConfig spiConfig;     ///< SPI config (for TMC2240)
#endif
};

/**
 * @brief Encoder configuration for position feedback
 */
struct EncoderSettings {
    uint8_t pinA;        ///< Encoder channel A pin
    uint8_t pinB;        ///< Encoder channel B pin
    uint8_t pinZ;        ///< Index pulse pin (optional)
    uint8_t attachMode;  ///< Counting mode: 1=Single, 2=HalfQuad, 4=FullQuad
    uint32_t resolution; ///< Encoder counts per revolution
};

/**
 * @brief Complete stepper motor configuration
 */
struct StepperConfig {
    String name;             ///< User-defined identifier for the stepper

    // Pin assignments
    uint8_t stepPin;         ///< GPIO for step pulses
    uint8_t dirPin;          ///< GPIO for direction signal
    uint8_t enablePin;       ///< GPIO for driver enable
    bool invertDirection;    ///< true to invert direction logic
    bool stepperEnabledHigh; ///< true if enable pin is active HIGH

    EncoderSettings encoderSettings;      ///< Encoder configuration
    StepperDriverSettings driverSettings; ///< Driver configuration
    
    // Motor parameters
    uint16_t stepsPerRev;    ///< Full steps per revolution (typically 200)
    uint16_t microsteps;     ///< Microstepping divisor (1-256)
    uint16_t rmsCurrent;     ///< RMS current limit in mA
    
    // Movement parameters
    double maxRPM;           ///< Maximum speed in RPM
    double rpsAcceleration;  ///< Acceleration in rotations/second²
    
    // LEDC configuration
    uint8_t ledcChannel;     ///< LEDC channel (0-15)

    float encoderToMicrostepRatio; ///< Computed ratio (do not set manually)
};

/**
 * @brief Real-time stepper status information
 */
struct StepperStatus {
    bool isInitialized;       ///< true if stepper is configured and ready
    bool isEnabled;           ///< true if driver is energized
    bool isMoving;            ///< true if currently moving
    int32_t currentPosition;  ///< Current position in microsteps
    int32_t targetPosition;   ///< Target position in microsteps
    double currentFrequency;  ///< Current step frequency in Hz
    double targetFrequency;   ///< Target step frequency in Hz
    uint32_t totalSteps;      ///< Total steps since startup
    bool stallGuard;          ///< true if stall detected (TMC drivers only)
    
    StepperStatus() {
        isInitialized = false;
        isEnabled = false;
        isMoving = false;
        currentPosition = 0;
        targetPosition = 0;
        currentFrequency = 0.0;
        targetFrequency = 0.0;
        totalSteps = 0;
        stallGuard = false;
    }
};

/**
 * @brief High-level multi-stepper controller with TMC driver integration
 * 
 * Manages up to MAX_STEPPERS (4) stepper motors with unified interface for:
 * - Position-based movement (absolute and relative)
 * - Velocity-based movement (frequency and angular speed)
 * - TMC driver configuration (microsteps, current, StallGuard)
 * - Encoder feedback integration
 */
class HighFrequencyStepper {
private:
    PWMStepper* pwmSteppers[MAX_STEPPERS];
    ESP32Encoder* pulseCounters[MAX_STEPPERS];
    TMC2209Stepper* tmc2209Drivers[MAX_STEPPERS];
#ifdef TMC2240
    TMC2240Stepper* tmc2240Drivers[MAX_STEPPERS];
#endif
    HardwareSerial* uartPorts[MAX_STEPPERS];
    
    StepperConfig configs[MAX_STEPPERS];
    StepperStatus status[MAX_STEPPERS];
    
    uint8_t stepperCount;
    bool globalEnable;
    
    bool validateStepperIndex(uint8_t index) const;
    void updatePosition(uint8_t index);
    
public:
    /** @brief Default constructor */
    HighFrequencyStepper();
    
    /** @brief Destructor - frees all allocated resources */
    ~HighFrequencyStepper();
    
    // ==================== Initialization ====================
    
    /**
     * @brief Add a stepper motor configuration
     * @param index Stepper index (0 to MAX_STEPPERS-1)
     * @param config Configuration struct with pin assignments and parameters
     * @return true if added successfully, false if index out of range
     */
    bool addStepper(uint8_t index, const StepperConfig& config);
    
    /**
     * @brief Initialize a configured stepper
     * @param index Stepper index
     * @return true if initialized successfully
     * @note Must call addStepper() first. Configures GPIOs, encoder, and TMC driver.
     */
    bool initializeStepper(uint8_t index);
    
    /**
     * @brief Initialize all configured steppers
     * @return true if all steppers initialized successfully
     */
    bool initializeAll();
    
    // ==================== Configuration Getters/Setters ====================
    
    /**
     * @brief Set maximum RPM
     * @param index Stepper index
     * @param rpm Maximum rotations per minute
     * @return true on success
     */
    bool setMaxRPM(uint8_t index, double rpm);
    
    /**
     * @brief Set acceleration rate
     * @param index Stepper index
     * @param rpsAcceleration Acceleration in rotations per second²
     * @return true on success
     */
    bool setAcceleration(uint8_t index, double rpsAcceleration);
    
    /**
     * @brief Set stepper name
     * @param index Stepper index
     * @param name User-defined name string
     * @return true on success
     */
    bool setName(uint8_t index, const String& name);
    
    /** @brief Get stepper name */
    String getName(uint8_t index) const;
    
    /** @brief Get step GPIO pin */
    uint8_t getStepPin(uint8_t index) const;
    
    /** @brief Get direction GPIO pin */
    uint8_t getDirPin(uint8_t index) const;
    
    /** @brief Get enable GPIO pin */
    uint8_t getEnablePin(uint8_t index) const;
    
    /** @brief Get encoder channel A pin */
    uint8_t getEncoderPinA(uint8_t index) const;
    
    /** @brief Get encoder channel B pin */
    uint8_t getEncoderPinB(uint8_t index) const;
    
    /** @brief Get encoder index (Z) pin */
    uint8_t getEncoderPinZ(uint8_t index) const;
    
    /** @brief Get total microsteps per revolution (stepsPerRev * microsteps) */
    uint16_t getMicrostepsPerRevolution(uint8_t index) const;
    
    /** @brief Get maximum step frequency in Hz */
    double getMaxFrequency(uint8_t index) const;
    
    /** @brief Get maximum RPM */
    double getMaxRPM(uint8_t index) const;
    
    /** @brief Get acceleration in rotations/s² */
    double getAcceleration(uint8_t index) const;
    
    /** @brief Get direction inversion setting */
    bool getInvertDirection(uint8_t index) const;

    // ==================== TMC2209 Driver Settings ====================
    
    /**
     * @brief Set microstepping resolution
     * @param index Stepper index
     * @param microsteps Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256)
     * @return true on success (TMC driver only)
     */
    bool setMicrosteps(uint8_t index, uint16_t microsteps);
    
    /**
     * @brief Set motor RMS current
     * @param index Stepper index
     * @param currentMA Current in milliamps
     * @return true on success (TMC driver only)
     */
    bool setRMSCurrent(uint8_t index, uint16_t currentMA);
    
    /**
     * @brief Enable/disable SpreadCycle mode
     * @param index Stepper index
     * @param enable true for SpreadCycle, false for StealthChop
     * @return true on success
     */
    bool setSpreadCycle(uint8_t index, bool enable);
    
    /** @brief Check if SpreadCycle is enabled */
    bool isSpreadCycleEnabled(uint8_t index) const;
    
    /**
     * @brief Set hybrid threshold for automatic mode switching
     * @param index Stepper index
     * @param threshold Velocity threshold (TSTEP units)
     * @return true on success
     */
    bool setHybridThreshold(uint8_t index, uint8_t threshold);
    
    /**
     * @brief Configure CoolStep current scaling
     * @param index Stepper index
     * @param value CoolStep value (0 = disabled)
     * @return true on success
     */
    bool setCoolStep(uint8_t index, uint16_t value);
    
    /**
     * @brief Set StallGuard threshold for stall detection
     * @param index Stepper index
     * @param threshold StallGuard sensitivity (0-255, lower = more sensitive)
     * @return true on success
     */
    bool setStallGuardThreshold(uint8_t index, uint16_t threshold);

    /** @brief Get current microstep setting */
    uint16_t getMicrosteps(uint8_t index) const;
    
    /** @brief Get RMS current setting in mA */
    uint16_t getRMSCurrent(uint8_t index) const;
    
    /** @brief Get sense resistor value in Ohms */
    float getRSense(uint8_t index) const;
    
    /** @brief Get UART port pointer (for TMC2209) */
    HardwareSerial* getUART(uint8_t index) const;
    
    /** @brief Get TMC driver address (0-3) */
    uint8_t getDriverAddress(uint8_t index) const;

    // ==================== Position-Based Movement ====================
    
    /**
     * @brief Move to absolute position
     * @param index Stepper index
     * @param position Target position in microsteps
     * @param frequency Maximum frequency (0 = use configured maxRPM)
     * @param blocking true to wait for completion
     * @return true if command accepted
     */
    bool moveToPosition(uint8_t index, int32_t position, double frequency = 0, bool blocking = true);
    
    /**
     * @brief Move relative to current position
     * @param index Stepper index
     * @param steps Steps to move (positive = forward)
     * @param frequency Maximum frequency (0 = use configured maxRPM)
     * @param blocking true to wait for completion
     * @return true if command accepted
     */
    bool moveRelative(uint8_t index, int32_t steps, double frequency = 0, bool blocking = true);
    
    /**
     * @brief Move to absolute angle
     * @param index Stepper index
     * @param angleDegrees Target angle in degrees
     * @param frequency Maximum frequency (0 = use configured maxRPM)
     * @param blocking true to wait for completion
     * @return true if command accepted
     */
    bool moveToAngle(uint8_t index, double angleDegrees, double frequency = 0, bool blocking = true);
    
    /**
     * @brief Move relative angle
     * @param index Stepper index
     * @param angleDegrees Degrees to rotate (positive = forward)
     * @param frequency Maximum frequency (0 = use configured maxRPM)
     * @param blocking true to wait for completion
     * @return true if command accepted
     */
    bool moveToAngleRelative(uint8_t index, double angleDegrees, double frequency = 0, bool blocking = true);

    // ==================== Velocity-Based Movement ====================
    
    /**
     * @brief Accelerate to target frequency
     * @param index Stepper index
     * @param frequency Target frequency in Hz (negative = reverse)
     * @param waitForCompletion true to block until target reached
     * @return true if command accepted
     */
    bool accelerateToFrequency(uint8_t index, double frequency, bool waitForCompletion = false);
    
    /**
     * @brief Accelerate to angular speed
     * @param index Stepper index
     * @param angularSpeed Target speed in degrees/second (negative = reverse)
     * @param waitForCompletion true to block until target reached
     * @return true if command accepted
     */
    bool accelerateToAngularSpeed(uint8_t index, double angularSpeed, bool waitForCompletion = false);
    
    /**
     * @brief Move at frequency immediately (no acceleration)
     * @param index Stepper index
     * @param frequency Target frequency in Hz (negative = reverse)
     * @return true if command accepted
     */
    bool moveAtFrequency(uint8_t index, double frequency);
    
    /**
     * @brief Move at angular speed immediately (no acceleration)
     * @param index Stepper index
     * @param angularSpeed Speed in degrees/second (negative = reverse)
     * @return true if command accepted
     */
    bool moveAtAngularSpeed(uint8_t index, double angularSpeed);

    // ==================== Stop Methods ====================
    
    /**
     * @brief Stop stepper with deceleration
     * @param index Stepper index
     * @return true if command accepted
     */
    bool stop(uint8_t index);
    
    /**
     * @brief Stop all steppers with deceleration
     * @return true if all stopped successfully
     */
    bool stopAll();
    
    /**
     * @brief Emergency stop - immediate halt of all steppers
     * @return true if stopped successfully
     * @warning No deceleration - may cause missed steps
     */
    bool emergencyStop();
    
    // ==================== Position & Status ====================
    
    /**
     * @brief Get current position from encoder
     * @param index Stepper index
     * @return Current position in microsteps
     */
    int32_t getPosition(uint8_t index);
    
    /**
     * @brief Get current angle
     * @param index Stepper index
     * @return Current angle in degrees
     */
    double getAngle(uint8_t index);
    
    /**
     * @brief Get target position
     * @param index Stepper index
     * @return Target position in microsteps
     */
    int32_t getTargetPosition(uint8_t index);
    
    /**
     * @brief Check if stepper is currently moving
     * @param index Stepper index
     * @return true if moving
     */
    bool isMoving(uint8_t index);
    
    /**
     * @brief Check if stepper is at target position
     * @param index Stepper index
     * @param tolerance Acceptable error in microsteps
     * @return true if within tolerance of target
     */
    bool isAtPosition(uint8_t index, int32_t tolerance = 1);
    
    /**
     * @brief Get current step frequency
     * @param index Stepper index
     * @return Current frequency in Hz
     */
    double getCurrentFrequency(uint8_t index);

    // ==================== Enable/Disable ====================
    
    /**
     * @brief Enable stepper driver (energize coils)
     * @param index Stepper index
     * @return true on success
     */
    bool enableStepper(uint8_t index);
    
    /**
     * @brief Disable stepper driver (de-energize coils)
     * @param index Stepper index
     * @return true on success
     */
    bool disableStepper(uint8_t index);
    
    /**
     * @brief Enable all configured steppers
     * @return true if all enabled successfully
     */
    bool enableAll();
    
    /**
     * @brief Disable all configured steppers
     * @return true if all disabled successfully
     */
    bool disableAll();
    
    /**
     * @brief Check if stepper is enabled
     * @param index Stepper index
     * @return true if enabled
     */
    bool isEnabled(uint8_t index);
    
    // ==================== Position Control ====================
    
    /**
     * @brief Set current position (resets encoder count)
     * @param index Stepper index
     * @param position New position value in microsteps
     * @return true on success
     */
    bool setPosition(uint8_t index, int32_t position);
    
    // ==================== Advanced TMC Features ====================
    
    /**
     * @brief Check if motor stall is detected
     * @param index Stepper index
     * @return true if stall detected (TMC driver with StallGuard only)
     */
    bool isStallDetected(uint8_t index);
    
    /**
     * @brief Check if stepper is using LEDC mode (high frequency)
     * @param index Stepper index
     * @return true if using LEDC, false if using timer mode
     */
    bool isInLEDCMode(uint8_t index);
    
    // ==================== Diagnostics ====================
    
    /**
     * @brief Get complete status structure
     * @param index Stepper index
     * @return StepperStatus struct with all current values
     */
    StepperStatus getStatus(uint8_t index);
    
    /**
     * @brief Print status to Serial
     * @param index Stepper index
     */
    void printStatus(uint8_t index);
    
    /**
     * @brief Print status of all steppers to Serial
     */
    void printAllStatus();
    
    /**
     * @brief Run self-test on stepper
     * @param index Stepper index
     * @return true if test passed
     */
    bool selfTest(uint8_t index);
    
    /**
     * @brief Run self-test on all steppers
     * @return true if all tests passed
     */
    bool selfTestAll();
    
    // ==================== Utility Methods ====================
    
    /** @brief Get number of configured steppers */
    uint8_t getStepperCount() const { return stepperCount; }
    
    /**
     * @brief Check if index is valid
     * @param index Stepper index to check
     * @return true if index is valid and stepper is configured
     */
    bool isValidIndex(uint8_t index) const { return validateStepperIndex(index); }
    
    /**
     * @brief Convert position to angle
     * @param index Stepper index
     * @param position Position in microsteps
     * @return Angle in degrees
     */
    double positionToAngle(uint8_t index, int32_t position);
    
    /**
     * @brief Convert angle to position
     * @param index Stepper index
     * @param angle Angle in degrees
     * @return Position in microsteps
     */
    int32_t angleToPosition(uint8_t index, double angle);
    
    /**
     * @brief Convert RPM to step frequency
     * @param index Stepper index
     * @param rpm Rotations per minute
     * @return Frequency in Hz
     */
    double rpmToFrequency(uint8_t index, double rpm) const;
    
    /**
     * @brief Convert step frequency to RPM
     * @param index Stepper index
     * @param frequency Frequency in Hz
     * @return Rotations per minute
     */
    double frequencyToRPM(uint8_t index, double frequency) const;
    
    /**
     * @brief Get configuration struct
     * @param index Stepper index
     * @return Copy of StepperConfig
     */
    StepperConfig getConfig(uint8_t index) const;
};

#endif // HIGHFREQUENCYSTEPPER_H