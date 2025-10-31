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

// Include organized test files
#include "test/TestUtils.h"
#include "test/DirectionTests.h"
#include "test/SpeedTests.h"
#include "test/OverflowTests.h"
#include "test/DemoTests.h"
#include "test/AngleTests.h"
#include "test/MaxSpeedTest.h"

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
    RESET_POSITION = 0
};

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    delay(2000);
    
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
    configTMC2209.uart = &Serial2;             // UART for TMC
    configTMC2209.driverAddress = 0b00;   // TMC2209 address
    configTMC2209.rSense = 0.11f;         // Current sense resistor
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
    configN23.uart = NULL;                   // The same UART for TMC
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
    nema17_enc.uart = NULL;                   // The same UART for TMC
    nema17_enc.microsteps = 32;        // Different microsteps
    nema17_enc.stepsPerRev = 200;
    nema17_enc.maxRPM = 2000;  // 2000 RPM max frequency
    nema17_enc.acceleration = 10000.0;          // Different acceleration
    nema17_enc.ledcChannel = 2;        // Different LEDC channel

    // Add steppers to controller
    // if (!stepperController.addStepper(TMC2209, configTMC2209)) {
    //     Serial.println("Failed to add stepper 0");
    //     return;
    // }

    if (!stepperController.addStepper(0, configN23)) {
        Serial.println("Failed to add stepper 1");
        return;
    }

    if (!stepperController.addStepper(1, nema17_enc)) {
        Serial.println("Failed to add stepper 2");
        return;
    }

    // Initialize all steppers
    if (!stepperController.initializeAll()) {
        Serial.println("Failed to initialize steppers");
        return;
    }
    
    // Enable all steppers
    stepperController.enableAll();
    
    // Print initial status
    stepperController.printAllStatus();
    
    // Perform self-test
    stepperController.selfTestAll();
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
    Serial.println("0. Reset Position Counter");
    Serial.println("\nEnter test number (or 'h' for help): ");
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
    processSerialInput();
    
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