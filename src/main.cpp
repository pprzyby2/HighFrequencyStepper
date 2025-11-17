/*
 * ESP32 Stepper Motor Test Suite
 * Comprehensive testing for PWMStepper + PulseCounter + TMC2209 integration
 * 
 * This example demonstrates:
 * - High-speed PWM step generation (up to 400kHz)
 * - Precision position tracking with PCNT
 * - TMC2209 stepper driver integration
 * - Organized test suite with separate test files
 */

#include <Arduino.h>
#include "PWMStepper.h"
#include "HighFrequencyStepper.h"
#include <TMCStepper.h>
#include "LEDStatusIndicator.h"

// Include organized test files
#include "test/TestUtils.h"
#include "test/DirectionTests.h"
#include "test/SpeedTests.h"
#include "test/OverflowTests.h"
#include "test/DemoTests.h"
#include "test/AngleTests.h"
#include "test/MaxSpeedTest.h"
#include <SPI.h>

// Pin definitions
#define EN_PIN           23          // Enable - PURPLE
#define DIR_PIN          21          // Direction - WHITE  
#define STEP_PIN         19          // Step - ORANGE
#define STEP_CNT_PIN     22          // Step counter input
#define SW_TX            17          // TMC2209 TX - BROWN
#define SW_RX            16          // TMC2209 RX - YELLOW
#define DRIVER_ADDRESS   0b00        // TMC2209 Driver address
#define R_SENSE          0.11f       // SilentStepStick series use 0.11

// N23 stepper pins (second stepper)
#define N23_EN_PIN       15          // N23 Enable pin - BLUE
#define N23_DIR_PIN      26          // N23 Direction pin - GREEN
#define N23_STEP_PIN     27           // N23 Step pin - RED
#define N23_CNT_A_PIN    25          // N23 Step counter input A
#define N23_CNT_B_PIN    33          // N23 Step counter input B

#define N23_2_EN_PIN       18          // N23 Enable pin - BLUE
#define N23_2_DIR_PIN      19          // N23 Direction pin - GREEN
#define N23_2_STEP_PIN     14           // N23 Step pin - RED
#define N23_2_CNT_A_PIN    32          // N23 Step counter input A
#define N23_2_CNT_B_PIN    35          // N23 Step counter input B

// HighFrequencyStepper controller
HighFrequencyStepper stepperController;

// LED Status Indicator (RGB LED on pin 48 for ESP32-S3)
LEDStatusIndicator ledStatus(48, 50);  // Pin 48, brightness 50/255

// Function declarations
void printTestMenu();
void runAllTests();
void processSerialInput();

enum StepperID {
    TMC2209 = 0,
    N23 = 1
};

// Test menu options
enum TestOption {
    TEST_BASIC_DIRECTION = 1,
    TEST_HIGH_SPEED = 2,
    TEST_LOW_SPEED = 3,
    TEST_OVERFLOW = 4,
    DEMO_POSITION_TRACKING = 5,
    DEMO_CLOSED_LOOP = 6,
    DEMO_SPEED_MEASUREMENT = 7,
    TEST_ANGLE_PRECISION = 8,
    TEST_MAX_SPEED = 9,
    TEST_ASYNC_MOVEMENT = 10,
    RUN_ALL_TESTS = 11,
    SHOW_SYSTEM_STATUS = 12,
    LED_STATUS_TEST = 13,
    MANUAL_CONTROL = 14,
    RESET_POSITION = 0
};

// Global variables for manual control mode
bool manualControlMode = false;
uint8_t manualControlStepperIndex = 0;
double manualControlSpeed = 100.0;  // Hz
bool manualControlDirection = true;  // true = forward

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, /*RX=*/18, /*TX=*/17);
    delay(5000);
    
    Serial.println("=== HighFrequencyStepper Example ===");
    
    // Configuration for Stepper 0
    StepperConfig configTMC2209;
    configTMC2209.stepPin = STEP_PIN;           // Step pin
    configTMC2209.dirPin = DIR_PIN;            // Direction pin
    configTMC2209.enablePin = EN_PIN;         // Enable pin
    configTMC2209.invertDirection = true; // Reverse counting
    configTMC2209.encoderAPin = STEP_CNT_PIN;      // Pulse counter pin
    configTMC2209.encoderBPin = DIR_PIN;      // Pulse counter pin
    configTMC2209.encoderAttachMode = 1;       // Default to Full Quad
    configTMC2209.encoderResolution = 200 * 256;    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
    configTMC2209.driverSettings.driverType = TMC2209_DRIVER;
    configTMC2209.driverSettings.uartConfig.uart = &Serial2;             // UART for TMC
    configTMC2209.driverSettings.uartConfig.driverAddress = 0b00;   // TMC220 9 address
    configTMC2209.driverSettings.uartConfig.rSense = 0.11f;         // Current sense resistor
    configTMC2209.microsteps = 256;        // 256 microsteps
    configTMC2209.rmsCurrent = 800;       // 800mA RMS current
    configTMC2209.stepsPerRev = 200;      // 200 steps per revolution
    configTMC2209.maxRPM = 360;  // 360 RPM max frequency
    configTMC2209.ledcChannel = 0;        // LEDC channel 0
    configTMC2209.name = "TMC2209";

    // Configuration for Stepper 1 (example second stepper)
    StepperConfig configN23;
    configN23.stepPin = N23_STEP_PIN;           // Different step pin
    configN23.dirPin = N23_DIR_PIN;             // Different direction pin
    configN23.enablePin = N23_EN_PIN;         // Can share enable pin
    configN23.stepperEnabledHigh = true;       // Active HIGH
    configN23.encoderAPin = N23_CNT_A_PIN;       // Different pulse counter pin
    configN23.encoderBPin = N23_CNT_B_PIN;       // Different pulse counter pin
    configN23.encoderAttachMode = 4;           // Default to Full Quad
    configN23.encoderResolution = 200;         // Default 200 CPR
    configN23.invertDirection = false;           // Reverse counting
    configN23.driverSettings.driverType = DriverType::STEP_DIR_ONLY;
    configN23.microsteps = 32;        // Different microsteps
    configN23.stepsPerRev = 200;
    configN23.maxRPM = 360;  // 360 RPM max frequency
    configN23.acceleration = 10000.0;          // Different acceleration
    configN23.ledcChannel = 1;        // Different LEDC channel
    configN23.name = "NEMA23+Encoder";

    /* StepperConfig configN23_2;
    configN23_2.stepPin = N23_2_STEP_PIN;           // Different step pin
    configN23_2.dirPin = N23_2_DIR_PIN;             // Different direction pin
    configN23_2.enablePin = N23_2_EN_PIN;         // Can share enable pin
    configN23_2.stepperEnabledHigh = true;       // Active HIGH
    configN23_2.encoderAPin = N23_2_CNT_A_PIN;       // Different pulse counter pin
    configN23_2.encoderBPin = N23_2_CNT_B_PIN;       // Different pulse counter pin
    configN23_2.encoderAttachMode = 4;           // Default to Full Quad
    configN23_2.encoderToMicrostepRatio = 8;     // 8:1 ratio for N23
    configN23_2.encoderResolution = 200;    // 200 counts per revolution
    configN23_2.invertDirection = false;           // Reverse counting
    configN23_2.uart = NULL;                   // The same UART for TMC
    configN23_2.microsteps = 32;        // Different microsteps
    configN23_2.stepsPerRev = 200;
    configN23_2.maxRPM = 360;  // 360 RPM max frequency
    configN23_2.acceleration = 10000.0;          // Different acceleration
    configN23_2.ledcChannel = 2;        // Different LEDC channel
    configN23_2.name = "N23_2_stepper";*/

    StepperConfig nema17_enc;
    nema17_enc.name = "NEMA17+Encoder";
    nema17_enc.stepPin = N23_2_STEP_PIN;           // Different step pin
    nema17_enc.dirPin = N23_2_DIR_PIN;             // Different direction pin
    nema17_enc.invertDirection = true;           // Reverse counting
    nema17_enc.enablePin = N23_2_EN_PIN;         // Can share enable pin
    nema17_enc.stepperEnabledHigh = true;       // Active HIGH
    nema17_enc.encoderAPin = N23_2_CNT_A_PIN;       // Different pulse counter pin
    nema17_enc.encoderBPin = N23_2_CNT_B_PIN;       // Different pulse counter pin
    nema17_enc.encoderAttachMode = 4;           // Default to Full Quad
    nema17_enc.encoderResolution = 1000;         // Default 1000 CPR
    nema17_enc.driverSettings.driverType = DriverType::STEP_DIR_ONLY;
    nema17_enc.microsteps = 32;        // Different microsteps
    nema17_enc.stepsPerRev = 200;
    nema17_enc.maxRPM = 2000;  // 2000 RPM max frequency
    nema17_enc.acceleration = 10000.0;          // Different acceleration
    nema17_enc.ledcChannel = 2;        // Different LEDC channel

    StepperConfig configTMC2240;
    configTMC2240.name = "TMC2240+Encoder";
    configTMC2240.stepPin = 5;           // Step pin
    configTMC2240.dirPin = 9;            // Direction pin
    configTMC2240.enablePin = 14;         // Enable pin
    configTMC2240.stepperEnabledHigh = false; // Active HIGH
    configTMC2240.invertDirection = false; // Reverse counting
    configTMC2240.encoderAPin = 6;      // Encoder A pin
    configTMC2240.encoderBPin = 7;      // Encoder B pin
    configTMC2240.encoderZPin = 27;      // Encoder Z pin
    configTMC2240.encoderAttachMode = 4;       // Default to Full Quad
    configTMC2240.encoderResolution = 1000;    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
    configTMC2240.driverSettings.driverType = TMC2240_DRIVER;
    configTMC2240.driverSettings.spiConfig.pinCS = 10; // Chip select pin
    configTMC2240.driverSettings.spiConfig.pinMOSI = 11; // MOSI pin
    configTMC2240.driverSettings.spiConfig.pinMISO = 13; // MISO pin
    configTMC2240.driverSettings.spiConfig.pinSCK = 12;  // SCK pin
    configTMC2240.driverSettings.spiConfig.link_index = 0; // Link index
    configTMC2240.microsteps = 64;        // microsteps
    configTMC2240.rmsCurrent = 2000;       // RMS current (mA)
    configTMC2240.stepsPerRev = 200;      // 200 steps per revolution
    configTMC2240.maxRPM = 2500;  // Max rotation speed
    configTMC2240.ledcChannel = 1;        // LEDC channel
    configTMC2240.acceleration = 15000.0;          // Different acceleration    

    StepperConfig configEncTMC2209;
    configEncTMC2209.name = "TMC2209+Encoder I";
    configEncTMC2209.stepPin = 45; //2;           // Step pin
    configEncTMC2209.dirPin = 47; //42;            // Direction pin
    configEncTMC2209.enablePin = 1;         // Enable pin
    configEncTMC2209.stepperEnabledHigh = false; // Active HIGH
    configEncTMC2209.invertDirection = true; // Reverse counting
    configEncTMC2209.encoderAPin = 21; //38;      // Encoder A pin
    configEncTMC2209.encoderBPin = 20; //40;      // Encoder B pin
    configEncTMC2209.encoderZPin = 19; //41;      // Encoder Z pin
    configEncTMC2209.encoderAttachMode = 4;       // Default to Full Quad
    configEncTMC2209.encoderResolution = 1000;    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
    configEncTMC2209.driverSettings.driverType = TMC2209_DRIVER; //STEP_DIR_ONLY;
    configEncTMC2209.driverSettings.uartConfig.uart = &Serial1;             // UART for TMC
    configEncTMC2209.driverSettings.uartConfig.driverAddress = 0b11; //0b00; // 0b11;   // TMC220 9 address
    configEncTMC2209.driverSettings.uartConfig.rSense = 0.11f;         // Current sense resistor
    configEncTMC2209.microsteps = 64;        // microsteps
    configEncTMC2209.rmsCurrent = 800;       // RMS current (mA)
    configEncTMC2209.stepsPerRev = 200;      // 200 steps per revolution
    configEncTMC2209.maxRPM = 300;  // Max rotation speed
    configEncTMC2209.ledcChannel = 0;        // LEDC channel
    configEncTMC2209.acceleration = 15000.0;          // Different acceleration

    StepperConfig configEncTMC2209_2;
    configEncTMC2209_2.name = "TMC2209+Encoder II";
    configEncTMC2209_2.stepPin = 2;           // Step pin
    configEncTMC2209_2.dirPin = 42;            // Direction pin
    configEncTMC2209_2.enablePin = 1;         // Enable pin
    configEncTMC2209_2.stepperEnabledHigh = false; // Active HIGH
    configEncTMC2209_2.invertDirection = true; // Reverse counting
    configEncTMC2209_2.encoderAPin = 38;      // Encoder A pin
    configEncTMC2209_2.encoderBPin = 40;      // Encoder B pin
    configEncTMC2209_2.encoderZPin = 41;      // Encoder Z pin
    configEncTMC2209_2.encoderAttachMode = 4;       // Default to Full Quad
    configEncTMC2209_2.encoderResolution = 1000;    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
    configEncTMC2209_2.driverSettings.driverType = TMC2209_DRIVER; //STEP_DIR_ONLY;
    configEncTMC2209_2.driverSettings.uartConfig.uart = &Serial1;             // UART for TMC
    configEncTMC2209_2.driverSettings.uartConfig.driverAddress = 0b00;  // TMC220 9 address
    configEncTMC2209_2.driverSettings.uartConfig.rSense = 0.11f;         // Current sense resistor
    configEncTMC2209_2.microsteps = 64;        // microsteps
    configEncTMC2209_2.rmsCurrent = 800;       // RMS current (mA)
    configEncTMC2209_2.stepsPerRev = 200;      // 200 steps per revolution
    configEncTMC2209_2.maxRPM = 300;  // Max rotation speed
    configEncTMC2209_2.ledcChannel = 1;        // LEDC channel
    configEncTMC2209_2.acceleration = 15000.0;          // Different acceleration

    if (!stepperController.addStepper(0, configEncTMC2209)) {
        Serial.println("Failed to add stepper 0");
        return;
    }
    if (!stepperController.addStepper(1, configEncTMC2209_2)) {
        Serial.println("Failed to add stepper 1");
        return;
    }

    // Initialize all steppers
    if (!stepperController.initializeAll()) {
        Serial.println("Failed to initialize steppers");
        return;
    }
    
    stepperController.setSpreadCycle(0, true); // Enable SpreadCycle for stepper 0
    // Enable all steppers
    stepperController.enableAll();
    
    // Initialize LED Status Indicator
    Serial.println("\n=== Initializing LED Status Indicator ===");
    if (!ledStatus.begin()) {
        Serial.println("WARNING: LED Status Indicator failed to initialize");
    } else {
        Serial.println("LED Status Indicator ready");
        ledStatus.setStatus(LED_INITIALIZING);
        delay(1000);
        ledStatus.test(); // Run LED test sequence
    }
    
    // Print initial status
    stepperController.printAllStatus();
    
    // Perform self-test
    stepperController.selfTestAll();
    
    // Set initial LED status to idle
    ledStatus.setStatus(LED_IDLE);
}

void printTestMenu() {
    Serial.println("\n=== TEST MENU ===");
    Serial.println("1. Basic Direction Tests");
    Serial.println("2. High Speed Tests (up to 400kHz)");
    Serial.println("3. Low Speed Precision Tests");
    Serial.println("4. Counter Overflow Tests");
    Serial.println("5. Position Tracking Demo");
    Serial.println("6. Closed Loop Control Demo");
    Serial.println("7. Speed Measurement Demo");
    Serial.println("8. Angle Precision Test");
    Serial.println("9. Max Speed Test");
    Serial.println("10. Asynchronous Movement Test");
    Serial.println("11. Run ALL Tests");
    Serial.println("12. Show System Status");
    Serial.println("13. LED Status Test");
    Serial.println("14. Manual Motor Control");
    Serial.println("0. Reset Position Counter");
    Serial.println("\nEnter test number (or 'h' for help): ");
}

void printManualControlMenu() {
    Serial.println("\n=== MANUAL MOTOR CONTROL ===");
    Serial.printf("Stepper: %s (Index %d)\n", stepperController.getName(manualControlStepperIndex).c_str(), manualControlStepperIndex);
    
    // Check if TMC2209 driver
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    bool isTMC2209 = (config.driverSettings.driverType == TMC2209_DRIVER);
    
    Serial.printf("Speed: %.2f Hz (%.2f RPM)\n", stepperController.getCurrentFrequency(manualControlStepperIndex), 
        stepperController.getCurrentFrequency(manualControlStepperIndex) / stepperController.getMicrostepsPerRevolution(manualControlStepperIndex) * 60.0);
    Serial.printf("Direction: %s\n", manualControlDirection ? "Forward (CW)" : "Reverse (CCW)");
    Serial.printf("Status: %s\n", stepperController.isEnabled(manualControlStepperIndex) ? "ENABLED" : "DISABLED");
    Serial.printf("Moving: %s\n", stepperController.isMoving(manualControlStepperIndex) ? "YES" : "NO");
    Serial.printf("Position: %d (%.2f degrees)\n", stepperController.getPosition(manualControlStepperIndex), stepperController.getAngle(manualControlStepperIndex));
    
    if (isTMC2209) {
        Serial.printf("Driver: TMC2209 (RMS Current: %d mA)\n", stepperController.getRMSCurrent(manualControlStepperIndex));
        Serial.printf("SpreadCycle: %s\n", stepperController.isSpreadCycleEnabled(manualControlStepperIndex) ? "ENABLED" : "DISABLED");
        Serial.printf("Microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    }
    
    Serial.println("\n--- Commands ---");
    Serial.println("e - Enable stepper");
    Serial.println("d - Disable stepper");
    Serial.println("s - Start/Resume movement");
    Serial.println("x - Stop movement");
    Serial.println("r - Reverse direction");
    Serial.println("+ - Increase speed (10 Hz)");
    Serial.println("- - Decrease speed (10 Hz)");
    Serial.println("* - Increase speed (100 Hz)");
    Serial.println("/ - Decrease speed (100 Hz)");
    Serial.println("n - Select next stepper");
    Serial.println("p - Show position/status");
    Serial.println("z - Zero position");
    
    if (isTMC2209) {
        Serial.println("\n--- TMC2209 Driver Controls ---");
        Serial.println("c - Increase RMS current (+50 mA)");
        Serial.println("C - Decrease RMS current (-50 mA)");
        Serial.println("t - Toggle StealthChop/SpreadCycle");
        Serial.println("g - Set StallGuard threshold");
        Serial.println("m - Change microsteps");
        Serial.println("i - Show TMC2209 info");
    }
    
    Serial.println("\nq - Quit manual control");
    Serial.println("\nEnter command: ");
}

void manualControlEnableStepper() {
    if (stepperController.enableStepper(manualControlStepperIndex)) {
        Serial.printf("✓ Stepper %s ENABLED\n", stepperController.getName(manualControlStepperIndex).c_str());
        ledStatus.setStatus(LED_IDLE);
    } else {
        Serial.println("✗ Failed to enable stepper");
        ledStatus.setStatus(LED_ERROR);
    }
}

void manualControlDisableStepper() {
    if (stepperController.disableStepper(manualControlStepperIndex)) {
        Serial.printf("✓ Stepper %s DISABLED\n", stepperController.getName(manualControlStepperIndex).c_str());
        ledStatus.setStatus(LED_DISABLED);
    } else {
        Serial.println("✗ Failed to disable stepper");
    }
}

void manualControlStart() {
    if (!stepperController.isEnabled(manualControlStepperIndex)) {
        Serial.println("✗ Stepper is disabled. Enable it first (press 'e')");
        return;
    }
    
    if (stepperController.moveAtFrequency(manualControlStepperIndex, manualControlSpeed, manualControlDirection)) {
        Serial.printf("✓ Moving at %.2f Hz %s\n", manualControlSpeed, manualControlDirection ? "Forward" : "Reverse");
        ledStatus.setStatus(LED_MOVING);
    } else {
        Serial.println("✗ Failed to start movement");
        ledStatus.setStatus(LED_ERROR);
    }
}

void manualControlStop() {
    if (stepperController.stop(manualControlStepperIndex)) {
        Serial.println("✓ Motor STOPPED");
        ledStatus.setStatus(LED_IDLE);
    } else {
        Serial.println("✗ Failed to stop motor");
    }
}

void manualControlReverseDirection() {
    manualControlDirection = !manualControlDirection;
    Serial.printf("✓ Direction changed to %s\n", manualControlDirection ? "Forward (CW)" : "Reverse (CCW)");
    
    // If motor is moving, update movement with new direction
    if (stepperController.isMoving(manualControlStepperIndex)) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
        manualControlStart();
    }
}

void manualControlIncreaseSpeed(double increment) {
    double maxFreq = stepperController.getMaxFrequency(manualControlStepperIndex);
    manualControlSpeed += increment;
    
    if (manualControlSpeed > maxFreq) {
        manualControlSpeed = maxFreq;
        Serial.printf("⚠ Speed limited to maximum: %.2f Hz\n", maxFreq);
    } else {
        Serial.printf("✓ Speed increased to %.2f Hz\n", manualControlSpeed);
    }
    
    // If motor is moving, update speed
    if (stepperController.isMoving(manualControlStepperIndex)) {
        manualControlStart();
    }
}

void manualControlDecreaseSpeed(double decrement) {
    manualControlSpeed -= decrement;
    
    if (manualControlSpeed < 1.0) {
        manualControlSpeed = 1.0;
        Serial.println("⚠ Speed limited to minimum: 1.0 Hz");
    } else {
        Serial.printf("✓ Speed decreased to %.2f Hz\n", manualControlSpeed);
    }
    
    // If motor is moving, update speed
    if (stepperController.isMoving(manualControlStepperIndex)) {
        manualControlStart();
    }
}

void manualControlSelectNextStepper() {
    // Stop current motor
    stepperController.stop(manualControlStepperIndex);
    
    // Move to next stepper
    manualControlStepperIndex++;
    if (manualControlStepperIndex >= stepperController.getStepperCount()) {
        manualControlStepperIndex = 0;
    }
    
    Serial.printf("✓ Selected stepper %s (Index %d)\n", 
                  stepperController.getName(manualControlStepperIndex).c_str(), 
                  manualControlStepperIndex);
}

void manualControlShowStatus() {
    Serial.println("\n--- Stepper Status ---");
    stepperController.printStatus(manualControlStepperIndex);
}

void manualControlZeroPosition() {
    stepperController.setPosition(manualControlStepperIndex, 0);
    Serial.printf("✓ Position reset to 0 for stepper %s\n", 
                  stepperController.getName(manualControlStepperIndex).c_str());
}

void manualControlIncreaseRMSCurrent() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    uint16_t currentRMS = stepperController.getRMSCurrent(manualControlStepperIndex);
    uint16_t newRMS = currentRMS + 50;
    
    if (newRMS > 2000) {
        Serial.println("⚠ Current limited to 2000 mA (safety limit)");
        newRMS = 2000;
    }
    
    if (stepperController.setRMSCurrent(manualControlStepperIndex, newRMS)) {
        Serial.printf("✓ RMS Current increased to %d mA\n", newRMS);
    } else {
        Serial.println("✗ Failed to set RMS current");
    }
}

void manualControlDecreaseRMSCurrent() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    uint16_t currentRMS = stepperController.getRMSCurrent(manualControlStepperIndex);
    
    if (currentRMS <= 100) {
        Serial.println("⚠ Current at minimum (100 mA)");
        return;
    }
    
    uint16_t newRMS = currentRMS - 50;
    if (newRMS < 100) {
        newRMS = 100;
    }
    
    if (stepperController.setRMSCurrent(manualControlStepperIndex, newRMS)) {
        Serial.printf("✓ RMS Current decreased to %d mA\n", newRMS);
    } else {
        Serial.println("✗ Failed to set RMS current");
    }
}

void manualControlToggleStealthChop() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    // Stop motor first for mode change
    bool wasMoving = stepperController.isMoving(manualControlStepperIndex);
    if (wasMoving) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
    }
    
    // Toggle: SpreadCycle = false means StealthChop is active
    // We'll use a static variable to track current state
    static bool spreadCycleEnabled = stepperController.isSpreadCycleEnabled(manualControlStepperIndex);
    spreadCycleEnabled = !spreadCycleEnabled;
    
    if (stepperController.setSpreadCycle(manualControlStepperIndex, spreadCycleEnabled)) {
        Serial.printf("✓ Switched to %s mode\n", spreadCycleEnabled ? "SpreadCycle (high speed)" : "StealthChop (quiet)");
    } else {
        Serial.println("✗ Failed to change mode");
    }
    
    // Resume if it was moving
    if (wasMoving) {
        delay(100);
        manualControlStart();
    }
}

void manualControlSetStallGuardThreshold() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.println("\nEnter StallGuard threshold (0-255, higher = less sensitive):");
    Serial.println("Typical values: 0-10 for very sensitive, 50-100 for moderate, 200+ for low sensitivity");
    Serial.print("Current value (-1 = disabled): ");
    
    // Wait for input with timeout
    unsigned long startTime = millis();
    while (!Serial.available() && (millis() - startTime < 10000)) {
        delay(10);
    }
    
    if (!Serial.available()) {
        Serial.println("\n⚠ Timeout - cancelled");
        return;
    }
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    int threshold = input.toInt();
    
    if (threshold < -1 || threshold > 255) {
        Serial.println("✗ Invalid threshold (must be -1 to 255)");
        return;
    }
    
    if (stepperController.setStallGuardThreshold(manualControlStepperIndex, threshold)) {
        if (threshold == -1) {
            Serial.println("✓ StallGuard disabled");
        } else {
            Serial.printf("✓ StallGuard threshold set to %d\n", threshold);
        }
    } else {
        Serial.println("✗ Failed to set StallGuard threshold");
    }
}

void manualControlChangeMicrosteps() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.printf("\nCurrent microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    Serial.println("Enter new microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256):");
    
    // Wait for input with timeout
    unsigned long startTime = millis();
    while (!Serial.available() && (millis() - startTime < 10000)) {
        delay(10);
    }
    
    if (!Serial.available()) {
        Serial.println("\n⚠ Timeout - cancelled");
        return;
    }
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    uint16_t microsteps = input.toInt();
    
    // Validate power of 2 and in range
    bool valid = false;
    uint16_t validValues[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    for (int i = 0; i < 9; i++) {
        if (microsteps == validValues[i]) {
            valid = true;
            break;
        }
    }
    
    if (!valid) {
        Serial.println("✗ Invalid microsteps value");
        return;
    }
    
    // Stop motor before changing microsteps
    bool wasMoving = stepperController.isMoving(manualControlStepperIndex);
    if (wasMoving) {
        stepperController.stop(manualControlStepperIndex);
        delay(100);
    }
    
    if (stepperController.setMicrosteps(manualControlStepperIndex, microsteps)) {
        Serial.printf("✓ Microsteps changed to %d\n", microsteps);
        Serial.printf("   Steps per revolution: %d\n", config.stepsPerRev * microsteps);
    } else {
        Serial.println("✗ Failed to set microsteps");
    }
    
    // Resume if it was moving
    if (wasMoving) {
        delay(100);
        manualControlStart();
    }
}

void manualControlShowTMCInfo() {
    StepperConfig config = stepperController.getConfig(manualControlStepperIndex);
    if (config.driverSettings.driverType != TMC2209_DRIVER) {
        Serial.println("✗ Not a TMC2209 driver");
        return;
    }
    
    Serial.println("\n=== TMC2209 Driver Information ===");
    Serial.printf("Stepper: %s\n", stepperController.getName(manualControlStepperIndex).c_str());
    Serial.printf("RMS Current: %d mA\n", stepperController.getRMSCurrent(manualControlStepperIndex));
    Serial.printf("Microsteps: %d\n", stepperController.getMicrosteps(manualControlStepperIndex));
    Serial.printf("R_Sense: %.3f Ω\n", stepperController.getRSense(manualControlStepperIndex));
    Serial.printf("Driver Address: 0x%02X\n", stepperController.getDriverAddress(manualControlStepperIndex));
    Serial.printf("UART: %s\n", stepperController.getUART(manualControlStepperIndex) == &Serial2 ? "Serial2" : "Serial1");
    Serial.printf("Stall Detection: %s\n", stepperController.isStallDetected(manualControlStepperIndex) ? "ACTIVE" : "Inactive");
    Serial.println("================================\n");
}

void processManualControlInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) {
            return;
        }
        
        char cmd = input.charAt(0);
        
        switch (cmd) {
            case 'e':
            case 'E':
                manualControlEnableStepper();
                break;
                
            case 'd':
            case 'D':
                manualControlDisableStepper();
                break;
                
            case 's':
            case 'S':
                manualControlStart();
                break;
                
            case 'x':
            case 'X':
                manualControlStop();
                break;
                
            case 'r':
            case 'R':
                manualControlReverseDirection();
                break;
                
            case '+':
                manualControlIncreaseSpeed(10.0);
                break;
                
            case '-':
                manualControlDecreaseSpeed(10.0);
                break;
                
            case '*':
                manualControlIncreaseSpeed(100.0);
                break;
                
            case '/':
                manualControlDecreaseSpeed(100.0);
                break;
                
            case 'n':
            case 'N':
                manualControlSelectNextStepper();
                break;
                
            case 'p':
            case 'P':
                manualControlShowStatus();
                break;
                
            case 'z':
            case 'Z':
                manualControlZeroPosition();
                break;
                
            case 'c':
                manualControlIncreaseRMSCurrent();
                break;
                
            case 'C':
                manualControlDecreaseRMSCurrent();
                break;
                
            case 't':
            case 'T':
                manualControlToggleStealthChop();
                break;
                
            case 'g':
            case 'G':
                manualControlSetStallGuardThreshold();
                break;
                
            case 'm':
            case 'M':
                manualControlChangeMicrosteps();
                break;
                
            case 'i':
            case 'I':
                manualControlShowTMCInfo();
                break;
                
            case 'q':
            case 'Q':
                manualControlStop();
                manualControlMode = false;
                Serial.println("✓ Exiting manual control mode");
                ledStatus.setStatus(LED_IDLE);
                printTestMenu();
                return;
                
            case 'h':
            case 'H':
            case '?':
                printManualControlMenu();
                return;
                
            default:
                Serial.println("✗ Invalid command. Press 'h' for help.");
                break;
        }
        
        printManualControlMenu();
    }
}

void runAllTests() {
    Serial.println("\n============================================================");
    Serial.println("RUNNING COMPLETE TEST SUITE");
    Serial.println("============================================================");
    
    clearTestResults();
    
    // Run all tests in sequence
    Serial.println("\n[1/12] Running direction tests...");
    for (int i = 0; i < stepperController.getStepperCount(); i++) {
        testDirectionChanges(stepperController, i);
    }

    Serial.println("\n[2/12] Running high speed tests...");
    testHighSpeedAcceleration(stepperController);

    Serial.println("\n[3/12] Running low speed precision tests...");
    testLowSpeedPrecision(stepperController);

    Serial.println("\n[4/12] Running overflow tests...");
    testCounterOverflow(stepperController);

    Serial.println("\n[5/12] Running position tracking demo...");
    demonstratePositionTracking(stepperController);

    Serial.println("\n[6/12] Running closed loop demo...");
    demonstrateClosedLoopControl(stepperController);

    Serial.println("\n[7/12] Running speed measurement demo...");
    demonstrateSpeedMeasurement(stepperController);

    Serial.println("[8/12] Running angle precision test...");
    testAnglePrecision(stepperController);

    Serial.println("[9/12] Running max speed test...");
    testMaxSpeed(stepperController);

    Serial.println("[10/12] Running asynchronous movement test...");
    testAsyncMovement(stepperController);

    // Print comprehensive summary
    printTestSummary();
}

void processSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input == "h" || input == "help") {
            printTestMenu();
            return;
        }
        
        int testChoice = input.toInt();
        
        switch (testChoice) {
            case TEST_BASIC_DIRECTION:
                clearTestResults();
                for (int i = 0; i < stepperController.getStepperCount(); i++) {
                    testDirectionChanges(stepperController, i);
                }
                printTestSummary();
                break;
                
            case TEST_HIGH_SPEED:
                clearTestResults();
                testHighSpeedAcceleration(stepperController);
                printTestSummary();
                break;
                
            case TEST_LOW_SPEED:
                clearTestResults();
                testLowSpeedPrecision(stepperController);
                printTestSummary();
                break;
                
            case TEST_OVERFLOW:
                clearTestResults();
                testCounterOverflow(stepperController);
                printTestSummary();
                break;
                
            case DEMO_POSITION_TRACKING:
                clearTestResults();
                demonstratePositionTracking(stepperController);
                printTestSummary();
                break;
                
            case DEMO_CLOSED_LOOP:
                clearTestResults();
                demonstrateClosedLoopControl(stepperController);
                printTestSummary();
                break;
                
            case DEMO_SPEED_MEASUREMENT:
                clearTestResults();
                demonstrateSpeedMeasurement(stepperController);
                printTestSummary();
                break;
                
            case RUN_ALL_TESTS:
                runAllTests();
                break;
                
            case SHOW_SYSTEM_STATUS:
                printSystemStatus(stepperController);
                break;
                
            case LED_STATUS_TEST:
                Serial.println("\n=== Running LED Status Test ===");
                ledStatus.test();
                Serial.println("LED Status Test Complete");
                ledStatus.setStatus(LED_IDLE);
                break;
                
            case MANUAL_CONTROL:
                manualControlMode = true;
                manualControlStepperIndex = 0;
                Serial.println("\n✓ Entering Manual Control Mode");
                Serial.println("All motors stopped. Use commands to control.");
                stepperController.stopAll();
                ledStatus.setStatus(LED_IDLE);
                printManualControlMenu();
                return;  // Don't print test menu again
                
            case RESET_POSITION:
                for (int i = 0; i < stepperController.getStepperCount(); i++) {
                    stepperController.setPosition(i, 0);
                    Serial.printf("Position of stepper %s reset to %d\n", stepperController.getName(i).c_str(), stepperController.getPosition(i));
                }
                break;

            case TEST_ANGLE_PRECISION:
                clearTestResults();
                testAnglePrecision(stepperController);
                printTestSummary();
                break;

            case TEST_MAX_SPEED:
                clearTestResults();
                //optimizeForMaxSpeed(stepperController, 0); // Optimize stepper 0 for max speed
                testMaxSpeed(stepperController);
                printTestSummary();
                break;

            case TEST_ASYNC_MOVEMENT:
                clearTestResults();
                testAsyncMovement(stepperController);
                printTestSummary();
                break;

            default:
                Serial.println("Invalid option. Type 'h' for help.");
                break;
        }
        
        printTestMenu();
    }
}

void loop() {
    // Update LED status indicator (for blinking effects)
    ledStatus.update();
    
    // Update LED based on stepper status
    static unsigned long lastLEDUpdate = 0;
    if (millis() - lastLEDUpdate > 100) { // Update every 100ms
        ledStatus.updateFromAllSteppers(stepperController);
        lastLEDUpdate = millis();
    }
    
    // Handle manual control mode separately
    if (manualControlMode) {
        processManualControlInput();
    } else {
        processSerialInput();
    }
    
    // Optional: periodic status updates
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 30000) { // Every 30 seconds
        Serial.println("\n--- Periodic Status Update ---");
        for (int i = 0; i < stepperController.getStepperCount(); i++) {
            Serial.printf("Stepper %s position: %d\n", stepperController.getName(i).c_str(), stepperController.getPosition(i));
        }
        lastStatusUpdate = millis();
    }
    
    delay(100); // Small delay to prevent overwhelming serial
}