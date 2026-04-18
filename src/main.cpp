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
#include "test/RapidTargetChangeTests.h"
#include "test/ManualControl.h"
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
    TEST_RAPID_TARGET_CHANGES = 11,
    TEST_RAPID_OSCILLATION = 12,
    TEST_CHASING_TARGET = 13,
    TEST_LONG_RUN = 14,
    TEST_MULTI_MOTOR_SPEEDS = 15,
    RUN_ALL_TESTS = 16,
    SHOW_SYSTEM_STATUS = 17,
    LED_STATUS_TEST = 18,
    MANUAL_CONTROL = 19,
    RESET_POSITION = 0
};

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, /*RX=*/18, /*TX=*/17);
    delay(5000);
    
    Serial.println("=== HighFrequencyStepper Example ===");
    
    StepperConfig ra_stepper_config = {
        .name = String("RA Stepper"),
        .stepPin = 2,
        .dirPin = 42,
        .enablePin = 1,         // Enable pin
        .invertDirection = false, // Reverse counting
        .stepperEnabledHigh = false, // Active HIGH
        .encoderSettings = {
            .pinA = 38,
            .pinB = 40,
            .pinZ = 41,
            .attachMode = 4,       // Default to Full Quad
            .resolution = 1000    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
        },
        .driverSettings = {
            .driverType = TMC2209_DRIVER, //STEP_DIR_ONLY;
            .uartConfig = {
                .uart = &Serial1,             // UART for TMC
                .driverAddress = 0b00, // TMC220 9 address
                .rSense = 0.11f         // Current sense resistor
            }
        },
        .stepsPerRev = 200,      // 200 steps per revolution
        .microsteps = 64,        // microsteps
        .rmsCurrent = 1000,       // RMS current (mA)
        .maxRPM = 300,  // Max rotation speed
        .rpsAcceleration = 3.0,
        .ledcChannel = 0        // LEDC channel        
    };

    StepperConfig dec_stepper_config = {
        .name = String("DEC Stepper"),
        .stepPin = 45, //2;           // Step pin
        .dirPin = 47, //42;            // Direction pin
        .enablePin = 1,         // Enable pin
        .invertDirection = false, // Reverse counting
        .stepperEnabledHigh = false, // Active HIGH
        .encoderSettings = {
            .pinA = 21, //38;      // Encoder A pin
            .pinB = 20, //40;      // Encoder B pin
            .pinZ = 19, //41;      // Encoder Z pin
            .attachMode = 4,       // Default to Full Quad
            .resolution = 1000    // Not really encoder. Just connect output STEP pin to input CNT and count steps (200 steps/rev * 256 microsteps)
        },
        .driverSettings = {
            .driverType = TMC2209_DRIVER, //STEP_DIR_ONLY;
            .uartConfig = {
                .uart = &Serial1,             // UART for TMC
                .driverAddress = 0b11, // TMC220 9 address
                .rSense = 0.11f         // Current sense resistor
            }
        },
        .stepsPerRev = 200,      // 200 steps per revolution
        .microsteps = 64,        // microsteps
        .rmsCurrent = 1000,       // RMS current (mA)
        .maxRPM = 300,  // Max rotation speed
        .rpsAcceleration = 3.0,
        .ledcChannel = 1        // LEDC channel        
    };

    if (!stepperController.addStepper(0, ra_stepper_config)) {
        Serial.println("Failed to add stepper 0");
        return;
    }
    if (!stepperController.addStepper(1, dec_stepper_config)) {
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
        //delay(1000);
        //ledStatus.test(); // Run LED test sequence
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
    Serial.println("11. Rapid Target Change Test");
    Serial.println("12. Rapid Oscillation Test");
    Serial.println("13. Chasing Target Test");
    Serial.println("14. Long Run Test");
    Serial.println("15. Multi-Motor Independent Speeds (LEDC Timer Test)");
    Serial.println("16. Run ALL Tests");
    Serial.println("17. Show System Status");
    Serial.println("18. LED Status Test");
    Serial.println("19. Manual Motor Control");
    Serial.println("0. Reset Position Counter");
    Serial.println("\nEnter test number (or 'h' for help): ");
}

void runAllTests() {
    Serial.println("\n============================================================");
    Serial.println("RUNNING COMPLETE TEST SUITE");
    Serial.println("============================================================");
    
    clearTestResults();
    
    // Run all tests in sequence
    Serial.println("\n[1/14] Running direction tests...");
    for (int i = 0; i < stepperController.getStepperCount(); i++) {
        testDirectionChanges(stepperController, i);
    }

    Serial.println("\n[2/14] Running high speed tests...");
    testHighSpeedAcceleration(stepperController);

    Serial.println("\n[3/14] Running low speed precision tests...");
    testLowSpeedPrecision(stepperController);

    Serial.println("\n[4/14] Running overflow tests...");
    testCounterOverflow(stepperController);

    Serial.println("\n[5/14] Running position tracking demo...");
    demonstratePositionTracking(stepperController);

    Serial.println("\n[6/14] Running closed loop demo...");
    demonstrateClosedLoopControl(stepperController);

    Serial.println("\n[7/14] Running speed measurement demo...");
    demonstrateSpeedMeasurement(stepperController);

    Serial.println("[8/14] Running angle precision test...");
    testAnglePrecision(stepperController);

    Serial.println("[9/14] Running max speed test...");
    testMaxSpeed(stepperController);

    Serial.println("[10/14] Running asynchronous movement test...");
    testAsyncMovement(stepperController);

    Serial.println("[11/14] Running rapid target change tests...");
    testRapidTargetChanges(stepperController);

    Serial.println("[12/14] Running rapid oscillation test...");
    testRapidOscillation(stepperController);

    Serial.println("[13/14] Running chasing target test...");
    testChasingTarget(stepperController);

    Serial.println("[14/14] Running long run test...");
    testLongRun(stepperController);

    Serial.println("\n[15/15] Running multi-motor independent speeds test...");
    testMultiMotorIndependentSpeeds(stepperController);
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

            case TEST_RAPID_TARGET_CHANGES:
                clearTestResults();
                testRapidTargetChanges(stepperController);
                printTestSummary();
                break;

            case TEST_RAPID_OSCILLATION:
                clearTestResults();
                testRapidOscillation(stepperController);
                printTestSummary();
                break;

            case TEST_CHASING_TARGET:
                clearTestResults();
                testChasingTarget(stepperController);
                printTestSummary();
                break;

            case TEST_LONG_RUN:
                clearTestResults();
                testLongRun(stepperController);
                printTestSummary();
                break;

            case TEST_MULTI_MOTOR_SPEEDS:
                clearTestResults();
                testMultiMotorIndependentSpeeds(stepperController);
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