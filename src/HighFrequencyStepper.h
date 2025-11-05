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
    uint8_t encoderZPin;    // Pin for encoder zero pin (not used in current implementation)
    uint8_t encoderAttachMode; // Mode for attaching encoder (1 - Single edge, 2- HalfQuad, 4 - FullQuad)
    uint32_t encoderResolution; // Resolution of the encoder (counts per revolution)
    float encoderToMicrostepRatio; // Ratio of encoder counts to microsteps
    HardwareSerial* uart;    // Pointer to HardwareSerial instance for TMC communication
    
    // TMC2209 configuration
    uint8_t driverAddress;   // TMC2209 address (0b00, 0b01, 0b10, 0b11)
    float rSense;            // Current sense resistor value (typically 0.11)
    
    // Motor parameters
    uint16_t microsteps;     // Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256)
    uint16_t rmsCurrent;     // RMS current in mA
    uint16_t stepsPerRev;    // Steps per revolution (typically 200 for 1.8° motors)
    
    // PWM/Movement parameters
    double maxRPM;     // Maximum frequency in Hz
    double acceleration;     // Acceleration in steps/s²
    bool invertDirection;    // Invert direction pin logic
    
    // LEDC configuration
    uint8_t ledcChannel;     // LEDC channel (0-15)
    String name;               // Name identifier for the stepper
    

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
        encoderResolution = 1000; // Default 1000 CPR
        driverAddress = 0;
        rSense = 0.11f;
        microsteps = 16;
        rmsCurrent = 800;
        stepsPerRev = 200;
        maxRPM = 500.0;
        acceleration = 10000.0;
        invertDirection = false;
        ledcChannel = 0;
        name = "Stepper_unknown";
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
    bool setMaxRPM(uint8_t index, double rpm);
    bool setAcceleration(uint8_t index, double acceleration);
    bool setName(uint8_t index, const String& name);
    String getName(uint8_t index) const;
    uint8_t getStepPin(uint8_t index) const;
    uint8_t getDirPin(uint8_t index) const;
    uint8_t getEnablePin(uint8_t index) const;
    uint8_t getStepCountPin(uint8_t index) const;
    uint16_t getMicrostepsPerRevolution(uint8_t index) const;
    double getMaxFrequency(uint8_t index) const;
    double getMaxRPM(uint8_t index) const;
    double getAcceleration(uint8_t index) const;
    bool getInvertDirection(uint8_t index) const;

    // TMC2209 Specific Settings
    bool setMicrosteps(uint8_t index, uint16_t microsteps);
    bool setRMSCurrent(uint8_t index, uint16_t currentMA);
    bool setSpreadCycle(uint8_t index, bool enable);
    bool setHybridThreshold(uint8_t index, uint8_t threshold);
    bool setCoolStep(uint8_t index, uint16_t value);
    bool setStallGuardThreshold(uint8_t index, uint16_t threshold);

    uint16_t getMicrosteps(uint8_t index) const;
    uint16_t getRMSCurrent(uint8_t index) const;
    float getRSense(uint8_t index) const;
    HardwareSerial* getUART(uint8_t index) const;
    uint8_t getDriverAddress(uint8_t index) const;

    
    // Movement methods
    bool moveToPosition(uint8_t index, int32_t position, double frequency = 0, bool blocking = true);
    bool moveRelative(uint8_t index, int32_t steps, double frequency = 0, bool blocking = true);
    bool moveToAngle(uint8_t index, double angleDegrees, double frequency = 0, bool blocking = true);
    bool moveToAngleRelative(uint8_t index, double angleDegrees, double frequency = 0, bool blocking = true);
    bool accelerateToFrequency(uint8_t index, double frequency, bool direction, bool waitForCompletion = false);
    bool accelerateToAngularSpeed(uint8_t index, double angularSpeed, bool direction, bool waitForCompletion = false);
    bool moveAtFrequency(uint8_t index, double frequency, bool direction = true);
    bool moveAtAngularSpeed(uint8_t index, double angularSpeed, bool direction = true);
    bool stop(uint8_t index);
    bool stopAll();
    bool emergencyStop();
    
    // Position and status methods
    int32_t getPosition(uint8_t index);
    double getAngle(uint8_t index);
    int32_t getTargetPosition(uint8_t index);
    bool isMoving(uint8_t index);
    bool isAtPosition(uint8_t index, int32_t tolerance = 1);
    double getCurrentFrequency(uint8_t index);
    double toAngle(uint8_t index, int32_t position);
    int32_t toPosition(uint8_t index, double angle);

    // Enable/disable methods
    bool enableStepper(uint8_t index);
    bool disableStepper(uint8_t index);
    bool enableAll();
    bool disableAll();
    bool isEnabled(uint8_t index);
    
    // Position control
    bool setPosition(uint8_t index, int32_t position);
    
    // Advanced TMC features
    bool isStallDetected(uint8_t index);
    
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
};

#endif // HIGHFREQUENCYSTEPPER_H