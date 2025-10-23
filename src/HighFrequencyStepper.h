#ifndef HIGHFREQUENCYSTEPPER_H
#define HIGHFREQUENCYSTEPPER_H

#include <Arduino.h>
#include "PWMStepper.h"
#include <TMCStepper.h>
#include <ESP32Encoder.h>

// Maximum number of steppers supported
#define MAX_STEPPERS 4

// Configuration structure for each stepper
struct StepperConfig {
    // Pin assignments
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    bool stepperEnabledHigh; // true if enable pin is active HIGH
    uint8_t encoderAPin;    // Pin for pulse counting (can be same as stepPin)
    uint8_t encoderBPin;    // Pin for pulse counting (can be same as stepPin)
    uint8_t encoderAttachMode; // Mode for attaching encoder (1 - Single edge, 2- HalfQuad, 4 - FullQuad)
    uint32_t encoderToMicrostepRatio; // Ratio of encoder counts to microsteps
    HardwareSerial* uart;    // Pointer to HardwareSerial instance for TMC communication
    
    // TMC2209 configuration
    uint8_t driverAddress;   // TMC2209 address (0b00, 0b01, 0b10, 0b11)
    float rSense;            // Current sense resistor value (typically 0.11)
    
    // Motor parameters
    uint16_t microsteps;     // Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256)
    uint16_t rmsCurrent;     // RMS current in mA
    uint16_t stepsPerRev;    // Steps per revolution (typically 200 for 1.8° motors)
    
    // PWM/Movement parameters
    double maxFrequency;     // Maximum frequency in Hz
    double acceleration;     // Acceleration in steps/s²
    bool invertDirection;    // Invert direction pin logic
    
    // LEDC configuration
    uint8_t ledcChannel;     // LEDC channel (0-15)
    

    // Default constructor
    StepperConfig() {
        stepPin = 0;
        dirPin = 0;
        enablePin = 0;
        stepperEnabledHigh = false;
        encoderAPin = 0;
        encoderBPin = 0;
        encoderAttachMode = 1; // Default to Single edge
        encoderToMicrostepRatio = 1; // Default 1:1
        driverAddress = 0;
        rSense = 0.11f;
        microsteps = 16;
        rmsCurrent = 800;
        stepsPerRev = 200;
        maxFrequency = 100000.0;
        acceleration = 10000.0;
        invertDirection = false;
        ledcChannel = 0;
    }
};

// Status structure for each stepper
struct StepperStatus {
    bool isInitialized;
    bool isEnabled;
    bool isMoving;
    int32_t currentPosition;
    int32_t targetPosition;
    double currentFrequency;
    double targetFrequency;
    uint32_t totalSteps;
    float temperature;
    bool stallGuard;
    
    StepperStatus() {
        isInitialized = false;
        isEnabled = false;
        isMoving = false;
        currentPosition = 0;
        targetPosition = 0;
        currentFrequency = 0.0;
        targetFrequency = 0.0;
        totalSteps = 0;
        temperature = 0.0;
        stallGuard = false;
    }
};

class HighFrequencyStepper {
private:
    // Arrays to hold instances for each stepper
    PWMStepper* pwmSteppers[MAX_STEPPERS];
    ESP32Encoder* pulseCounters[MAX_STEPPERS];
    TMC2209Stepper* tmcDrivers[MAX_STEPPERS];
    HardwareSerial* uartPorts[MAX_STEPPERS];
    
    // Configuration and status for each stepper
    StepperConfig configs[MAX_STEPPERS];
    StepperStatus status[MAX_STEPPERS];
    
    // Internal state
    uint8_t stepperCount;
    bool globalEnable;
    
    // Helper methods
    bool validateStepperIndex(uint8_t index) const;
    void updatePosition(uint8_t index);
    
public:
    // Constructor
    HighFrequencyStepper();
    
    // Destructor
    ~HighFrequencyStepper();
    
    // Initialization methods
    bool addStepper(uint8_t index, const StepperConfig& config);
    bool initializeStepper(uint8_t index);
    bool initializeAll();
    
    // Configuration methods
    bool setMicrosteps(uint8_t index, uint16_t microsteps);
    bool setRMSCurrent(uint8_t index, uint16_t currentMA);
    bool setMaxFrequency(uint8_t index, double frequency);
    bool setAcceleration(uint8_t index, double acceleration);
    uint8_t getStepPin(uint8_t index) const;
    uint8_t getDirPin(uint8_t index) const;
    uint8_t getEnablePin(uint8_t index) const;
    uint8_t getStepCountPin(uint8_t index) const;
    HardwareSerial* getUART(uint8_t index) const;
    float getRSense(uint8_t index) const;
    uint8_t getDriverAddress(uint8_t index) const;
    uint16_t getMicrosteps(uint8_t index) const;
    uint16_t getRMSCurrent(uint8_t index) const;
    uint16_t getMicrostepsPerRevolution(uint8_t index) const;
    double getMaxFrequency(uint8_t index) const;
    double getAcceleration(uint8_t index) const;
    bool getInvertDirection(uint8_t index) const;
    
    // Movement methods
    bool moveToPosition(uint8_t index, int32_t position, double frequency = 0, bool blocking = true);
    bool moveRelative(uint8_t index, int32_t steps, double frequency = 0, bool blocking = true);
    bool accelerateToFrequency(uint8_t index, double frequency, bool direction, bool waitForCompletion = false);
    bool moveAtFrequency(uint8_t index, double frequency, bool direction = true);
    bool stop(uint8_t index);
    bool stopAll();
    bool emergencyStop();
    
    // Position and status methods
    int32_t getPosition(uint8_t index);
    int32_t getTargetPosition(uint8_t index);
    bool isMoving(uint8_t index);
    bool isAtPosition(uint8_t index, int32_t tolerance = 1);
    double getCurrentFrequency(uint8_t index);
    
    // Enable/disable methods
    bool enableStepper(uint8_t index);
    bool disableStepper(uint8_t index);
    bool enableAll();
    bool disableAll();
    bool isEnabled(uint8_t index);
    
    // Position control
    bool setPosition(uint8_t index, int32_t position);
    
    // Advanced TMC features
    bool enableStallGuard(uint8_t index, uint8_t threshold = 10);
    bool disableStallGuard(uint8_t index);
    bool isStallDetected(uint8_t index);
    float getTemperature(uint8_t index);
    uint16_t getStallGuardValue(uint8_t index);
    
    // PWM mode control
    bool isInLEDCMode(uint8_t index);
    
    // Diagnostic methods
    StepperStatus getStatus(uint8_t index);
    void printStatus(uint8_t index);
    void printAllStatus();
    bool selfTest(uint8_t index);
    bool selfTestAll();
    
    // Utility methods
    uint8_t getStepperCount() const { return stepperCount; }
    bool isValidIndex(uint8_t index) const { return validateStepperIndex(index); }
    
    // Configuration access
    StepperConfig getConfig(uint8_t index) const;
    
    // Synchronization methods
    bool moveAllToPosition(const int32_t positions[], double frequency = 0);
    bool moveAllRelative(const int32_t steps[], double frequency = 0);
    bool waitForCompletion(uint8_t index, uint32_t timeoutMS = 10000);
    bool waitForAllCompletion(uint32_t timeoutMS = 10000);
    
    // Advanced features
    bool setInterpolation(uint8_t index, bool enable);
    bool setSpreadCycle(uint8_t index, bool enable);
    bool setHybridThreshold(uint8_t index, uint8_t threshold);
    bool setCoolStep(uint8_t index, uint8_t semin, uint8_t semax, uint8_t sedn, uint8_t seimin);
};

#endif // HIGHFREQUENCYSTEPPER_H