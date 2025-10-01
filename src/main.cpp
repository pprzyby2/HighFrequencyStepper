/*
 * PWMStepper + PulseCounter Integration Example
 * Demonstrates position feedback using PCNT module
 * 
 * This example shows:
 * - PWM step generation with LEDC
 * - Position tracking with PCNT
 * - Closed-loop position control
 * - Speed measurement
 * - TMC2209 integration
 */

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"
#include <TMCStepper.h>
#include "driver/gpio.h"
#include "HighFrequencyStepper.h"

// Pin definitions (same as your main.cpp)
#define EN_PIN           23          // Enable - PURPLE
#define DIR_PIN          21          // Direction - WHITE  
#define STEP_PIN         19          // Step - ORANGE
#define STEP_CNT_PIN     22          // Step - ORANGE (for PulseCounter)
#define SW_TX            17          // SoftwareSerial receive pin - BROWN
#define SW_RX            16          // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS   0b00        // TMC2209 Driver address
#define R_SENSE          0.11f       // SilentStepStick series use 0.11

// Create instances
PWMStepper pwmStepper(STEP_PIN, DIR_PIN, EN_PIN, 0); // LEDC channel 0
PulseCounter pulseCounter(PCNT_UNIT_1, STEP_CNT_PIN, DIR_PIN); // Monitor step pin
TMC2209Stepper TMC_Driver(&Serial2, R_SENSE, DRIVER_ADDRESS);

// Test results tracking
struct TestResult {
    String testName;
    bool passed;
    String details;
    float accuracy;
};

TestResult testResults[20]; // Array to store test results
int testCount = 0;

void addTestResult(String name, bool passed, String details = "", float accuracy = 0.0) {
    if (testCount < 20) {
        testResults[testCount].testName = name;
        testResults[testCount].passed = passed;
        testResults[testCount].details = details;
        testResults[testCount].accuracy = accuracy;
        testCount++;
    }
}

void printTestSummary() {
    Serial.println("\n########################################");
    Serial.println("#          FINAL TEST SUMMARY          #");
    Serial.println("########################################");
    
    int passedTests = 0;
    int failedTests = 0;
    
    Serial.println("Test Name                    | Status | Details");
    Serial.println("---------------------------------------------|--------");
    
    for (int i = 0; i < testCount; i++) {
        String status = testResults[i].passed ? "PASS" : "FAIL";
        
        // Format test name (max 28 chars)
        String formattedName = testResults[i].testName;
        while (formattedName.length() < 28) {
            formattedName += " ";
        }
        if (formattedName.length() > 28) {
            formattedName = formattedName.substring(0, 25) + "...";
        }
        
        Serial.print(formattedName);
        Serial.print(" | ");
        Serial.print(status);
        Serial.print("   | ");
        
        if (testResults[i].accuracy > 0) {
            Serial.print(testResults[i].accuracy, 1);
            Serial.print("% - ");
        }
        Serial.println(testResults[i].details);
        
        if (testResults[i].passed) {
            passedTests++;
        } else {
            failedTests++;
        }
    }
    
    Serial.println("---------------------------------------------|--------");
    Serial.print("TOTAL TESTS: "); Serial.print(testCount);
    Serial.print(" | PASSED: "); Serial.print(passedTests);
    Serial.print(" | FAILED: "); Serial.println(failedTests);
    
    float successRate = (float)passedTests / testCount * 100.0;
    Serial.print("SUCCESS RATE: "); Serial.print(successRate, 1); Serial.println("%");
    
    Serial.println("\n########################################");
    if (failedTests == 0) {
        Serial.println("#         ALL TESTS PASSED! ✓          #");
    } else {
        Serial.println("#       SOME TESTS FAILED! ✗           #");
        Serial.println("#     CHECK SYSTEM CONFIGURATION       #");
    }
    Serial.println("########################################\n");
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    delay(1000);
    
    Serial.println("=== PWMStepper + PulseCounter Example ===");
    
    // Initialize PWM stepper
    pwmStepper.begin();
    
    // Initialize pulse counter
    pulseCounter.begin();
    //pulseCounter.enableInterrupt();  // Enable overflow handling
    pulseCounter.start();            // Start counting
    
    // Initialize TMC2209
    TMC_Driver.begin();
    TMC_Driver.toff(5);                 // Enable driver in software
    TMC_Driver.rms_current(2200);       // Set motor RMS current (mA)
    TMC_Driver.microsteps(256);         // Set microsteps
    TMC_Driver.VACTUAL(0);              // Switch off internal stepper control
    TMC_Driver.en_spreadCycle(true);   // Toggle spreadCycle
    TMC_Driver.pwm_autoscale(true);     // Needed for stealthChop

    // Print pin configuration
    Serial.println("Pin Configuration:");
    Serial.printf("STEP_PIN: %d, DIR_PIN: %d, STEP_CNT_PIN: %d, EN_PIN: %d\n", 
                 STEP_PIN, DIR_PIN, STEP_CNT_PIN, EN_PIN);
    Serial.println("Setup complete!");
    delay(2000);
}

void demonstratePositionTracking() {
    Serial.println("\n=== Position Tracking Demo ===");
    
    pulseCounter.resetPosition();
    
    // Move forward and track position
    Serial.println("Moving 1600 steps forward...");
    pwmStepper.enable();
    pwmStepper.setDirection(true);
    pwmStepper.startPWM(800);
    
    for (int i = 0; i < 20; i++) {
        delay(100);
        Serial.print("Position: "); 
        Serial.print(pulseCounter.getPosition());
        Serial.print(" | Speed: ");
        Serial.print(pulseCounter.getStepsPerSecond(500));
        Serial.println(" steps/sec");
    }
    
    pwmStepper.stopPWM();
    delay(1000);
    
    Serial.print("Final position: "); 
    Serial.println(pulseCounter.getPosition());
}

void demonstrateClosedLoopControl() {
    Serial.println("\n=== Closed-Loop Position Control ===");
    
    pulseCounter.resetPosition();
    int32_t targetPosition = 32000; // Target position
    uint32_t tolerance = 0;        // Position tolerance

    Serial.printf("Moving to position: %d\n", targetPosition);

    pwmStepper.enable();
    pwmStepper.setDirection(targetPosition > 0);
    pwmStepper.startPWM(1000);
    
    uint32_t startTime = millis();
    uint32_t timeout = 10000; // 10 second timeout
    
    while (millis() - startTime < timeout) {
        int32_t currentPos = pulseCounter.getPosition();
        int32_t error = targetPosition - currentPos;
        
        if (abs(error) <= tolerance) {
            Serial.println("Target reached!");
            break;
        }
        
        // Simple proportional control for speed
        uint32_t speed = constrain(abs(error) * 2, 100, 20000);
        pwmStepper.setFrequency(speed);
        
        // Update direction if needed
        bool newDirection = error > 0;
        if (newDirection != pwmStepper.getDirection()) {
            pwmStepper.setDirection(newDirection);
        }
        
        // Print status every 200ms
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 200) {
            Serial.print("Pos: "); Serial.print(currentPos);
            Serial.print(" | Target: "); Serial.print(targetPosition);
            Serial.print(" | Error: "); Serial.print(error);
            Serial.print(" | Speed: "); Serial.println(speed);
            lastPrint = millis();
        }
        
        delay(10);
    }
    
    pwmStepper.stopPWM();
    Serial.print("Final position: "); 
    Serial.print(pulseCounter.getPosition());
    Serial.print(" (Target: "); Serial.print(targetPosition); Serial.println(")");
}

void demonstrateSpeedMeasurement() {
    Serial.println("\n=== Speed Measurement Demo ===");
    
    pwmStepper.enable();
    
    // Test different speeds
    uint32_t testSpeeds[] = {200, 500, 1000, 1500, 2000};
    
    for (int i = 0; i < 5; i++) {
        uint32_t setSpeed = testSpeeds[i];
        Serial.print("Setting speed to: "); Serial.print(setSpeed); Serial.println(" Hz");
        
        pwmStepper.setDirection(i % 2 == 0); // Alternate direction
        pwmStepper.startPWM(setSpeed);
        
        delay(1000); // Let it stabilize
        
        // Measure speed over 2 seconds
        for (int j = 0; j < 10; j++) {
            int32_t measuredSpeed = pulseCounter.getStepsPerSecond(200);
            if (measuredSpeed != 0) {
                Serial.printf("Steps: %d Measured: %d Hz | Error: %d Hz\n", measuredSpeed, abs(measuredSpeed), abs(measuredSpeed) - setSpeed);
            }
            delay(200);
        }
        
        pwmStepper.stopPWM();
        delay(500);
    }
}

void printSystemStatus() {
    Serial.println("\n=== System Status ===");
    
    // PWM Stepper status
    Serial.println("PWM Stepper:");
    Serial.print("  Enabled: "); Serial.println(pwmStepper.isEnabled() ? "Yes" : "No");
    Serial.print("  Direction: "); Serial.println(pwmStepper.getDirection() ? "Forward" : "Reverse");
    Serial.print("  Frequency: "); Serial.print(pwmStepper.getFrequency()); Serial.println(" Hz");
    
    // Pulse Counter status
    Serial.println("Pulse Counter:");
    pulseCounter.printStatus();
    
    // TMC2209 status
    Serial.println("TMC2209:");
    Serial.print("  Connection: "); Serial.println(TMC_Driver.test_connection());
    Serial.print("  SG Result: "); Serial.println(TMC_Driver.SG_RESULT());
    Serial.print("  Current: "); Serial.print(TMC_Driver.cs2rms(TMC_Driver.cs_actual())); Serial.println(" mA");
}

void testDirectionChanges() {
    Serial.println("\n=== Direction Change Test ===");
    
    pulseCounter.resetPosition();
    pwmStepper.enable();
    
    // Test 1: Forward direction
    Serial.println("Testing FORWARD direction (1000 steps)...");
    pwmStepper.setDirection(true);
    delay(100); // Let direction settle
    
    int32_t startPos = pulseCounter.getPosition();
    pwmStepper.startPWM(800);
    delay(1250); // 1000 steps at 800 Hz = 1.25 seconds
    pwmStepper.stopPWM();
    
    int32_t forwardPos = pulseCounter.getPosition();
    int32_t forwardSteps = forwardPos - startPos;
    
    Serial.print("Forward steps counted: "); Serial.println(forwardSteps);
    Serial.print("Expected: ~1000, Actual: "); Serial.println(forwardSteps);
    Serial.print("Direction detection: "); Serial.println(pulseCounter.getDirection() ? "Forward" : "Reverse");
    
    // Evaluate forward test
    bool forwardTest = (forwardSteps >= 950 && forwardSteps <= 1050); // ±5% tolerance
    float forwardAccuracy = (float)forwardSteps / 1000.0 * 100.0;
    addTestResult("Forward Direction", forwardTest, 
                  "Expected: 1000, Got: " + String(forwardSteps), forwardAccuracy);
    
    delay(1000);
    
    // Test 2: Reverse direction
    Serial.println("Testing REVERSE direction (1000 steps)...");
    pwmStepper.setDirection(false);
    delay(100); // Let direction settle
    
    startPos = pulseCounter.getPosition();
    pwmStepper.startPWM(800);
    delay(1250); // 1000 steps at 800 Hz = 1.25 seconds
    pwmStepper.stopPWM();
    
    int32_t reversePos = pulseCounter.getPosition();
    int32_t reverseSteps = reversePos - startPos;
    
    Serial.print("Reverse steps counted: "); Serial.println(reverseSteps);
    Serial.print("Expected: ~-1000, Actual: "); Serial.println(reverseSteps);
    Serial.print("Direction detection: "); Serial.println(pulseCounter.getDirection() ? "Forward" : "Reverse");
    
    // Evaluate reverse test
    bool reverseTest = (reverseSteps <= -950 && reverseSteps >= -1050); // ±5% tolerance
    float reverseAccuracy = (float)abs(reverseSteps) / 1000.0 * 100.0;
    addTestResult("Reverse Direction", reverseTest, 
                  "Expected: -1000, Got: " + String(reverseSteps), reverseAccuracy);
    
    // Test 3: Multiple direction changes
    Serial.println("Testing rapid direction changes...");
    int passedChanges = 0;
    for (int i = 0; i < 5; i++) {
        bool dir = (i % 2 == 0);
        pwmStepper.setDirection(dir);
        delay(50);
        
        startPos = pulseCounter.getPosition();
        pwmStepper.startPWM(1000);
        delay(200); // 200 steps
        pwmStepper.stopPWM();
        
        int32_t steps = pulseCounter.getPosition() - startPos;
        int32_t expectedSteps = dir ? 200 : -200;
        bool changeOK = (abs(abs(steps) - 200) <= 20); // ±10% tolerance
        
        if (changeOK) passedChanges++;
        
        Serial.print("Change "); Serial.print(i+1); 
        Serial.print(" ("); Serial.print(dir ? "FWD" : "REV"); 
        Serial.print("): "); Serial.print(steps);
        Serial.println(changeOK ? " ✓" : " ✗");
        
        delay(100);
    }
    
    bool rapidChangesTest = (passedChanges >= 4); // At least 4/5 must pass
    addTestResult("Rapid Direction Changes", rapidChangesTest, 
                  String(passedChanges) + "/5 changes successful");
    
    Serial.print("Final position after direction test: "); 
    Serial.println(pulseCounter.getPosition());
}

void testCounterOverflow() {
    Serial.println("\n=== PCNT Counter Overflow Test ===");
    
    pulseCounter.resetPosition();
    
    pwmStepper.enable();
    pwmStepper.setDirection(true);
    
    Serial.println("Testing positive overflow (moving to +6000 steps)...");
    
    // Monitor position as we approach and exceed the limit
    pwmStepper.startPWM(20000); // High speed for faster test
    
    bool positiveOverflowOK = false;
    for (int i = 0; i < 60; i++) { // 6 seconds at 20kHz = 120000 steps
        delay(100);
        int32_t pos = pulseCounter.getPosition();
        int16_t raw = pulseCounter.getRawCount();
        
        Serial.print("Time: "); Serial.print(i * 100); Serial.print("ms");
        Serial.print(" | Raw: "); Serial.print(raw);
        Serial.print(" | Position: "); Serial.println(pos);
        
        if (pos > 60000) {
            Serial.println("Positive overflow test completed!");
            positiveOverflowOK = true;
            break;
        }
    }
    
    pwmStepper.stopPWM();
    addTestResult("Positive Overflow", positiveOverflowOK, 
                  positiveOverflowOK ? "Successfully exceeded +60000" : "Failed to reach +60000");
    delay(1000);
    
    Serial.println("Testing negative overflow (moving to -60000 steps)...");
    pwmStepper.setDirection(false);
    pwmStepper.startPWM(20000);
    
    bool negativeOverflowOK = false;
    for (int i = 0; i < 120; i++) { // More time to go negative
        delay(100);
        int32_t pos = pulseCounter.getPosition();
        int16_t raw = pulseCounter.getRawCount();
        
        Serial.print("Time: "); Serial.print(i * 100); Serial.print("ms");
        Serial.print(" | Raw: "); Serial.print(raw);
        Serial.print(" | Position: "); Serial.println(pos);
        
        if (pos < -60000) {
            Serial.println("Negative overflow test completed!");
            negativeOverflowOK = true;
            break;
        }
    }
    
    pwmStepper.stopPWM();
    addTestResult("Negative Overflow", negativeOverflowOK, 
                  negativeOverflowOK ? "Successfully exceeded -60000" : "Failed to reach -60000");
        
    Serial.print("Final position after overflow test: ");
    Serial.println(pulseCounter.getPosition());
}

void testHighSpeedAcceleration() {
    Serial.println("\n=== High Speed Acceleration Test (up to 200kHz) ===");
    
    pulseCounter.resetPosition();
    pwmStepper.enable();
    pwmStepper.setDirection(true);
    
    // Test speeds from 1kHz to 200kHz
    uint32_t testSpeeds[] = {1000, 5000, 10000, 25000, 50000, 75000, 100000, 150000, 200000, 250000, 300000, 350000, 400000}; // In my setup max is ~300kHz (using 256 microsteps)
    int numSpeeds = sizeof(testSpeeds) / sizeof(testSpeeds[0]);
    
    Serial.println("Testing acceleration profile...");
    
    int passedSpeeds = 0;
    for (int i = 0; i < numSpeeds; i++) {
        uint32_t speed = testSpeeds[i];
        Serial.print("Setting speed to: "); Serial.print(speed); Serial.println(" Hz");
        
        for (int acceleration = 0; acceleration <= speed; acceleration += 2560) {
            pwmStepper.startPWM(acceleration);
            Serial.print("  Acceleration: "); Serial.print(acceleration); Serial.println(" Hz/s");
            delay(100); // Let it stabilize
        }
        pwmStepper.startPWM(speed);
        int32_t startPos = pulseCounter.getPosition();
        uint32_t startTime = millis();        
        delay(1000); // Run for 1000ms at each speed
        pwmStepper.stopPWM();
        
        uint32_t endTime = millis();
        int32_t endPos = pulseCounter.getPosition();
        
        uint32_t duration = endTime - startTime;
        int32_t stepsCounted = endPos - startPos;
        uint32_t expectedSteps = (speed * duration) / 1000;
        float accuracy = (float)stepsCounted / expectedSteps * 100.0;
        
        Serial.print("  Duration: "); Serial.print(duration); Serial.print("ms");
        Serial.print(" | Expected steps: "); Serial.print(expectedSteps);
        Serial.print(" | Counted: "); Serial.print(stepsCounted);
        Serial.print(" | Accuracy: "); Serial.print(accuracy, 1); Serial.println("%");
        
        bool speedTestOK = (accuracy >= 90.0);
        if (speedTestOK) passedSpeeds++;
        
        String speedName = "Speed " + String(speed/1000) + "kHz";
        addTestResult(speedName, speedTestOK, 
                      "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                      accuracy);
        
        if (accuracy < 90.0) {
            Serial.println("  WARNING: Low accuracy detected!");
        }
        
        delay(200); // Brief pause between speeds
    }
    
    // Overall high speed test result
    bool overallHighSpeedOK = (passedSpeeds >= 7); // At least 7/9 speeds must pass
    addTestResult("High Speed Overall", overallHighSpeedOK, 
                  String(passedSpeeds) + "/" + String(numSpeeds) + " speeds passed");
    
    Serial.println("High speed test completed!");
    Serial.print("Total steps moved: "); Serial.println(pulseCounter.getPosition());
}

void testLowSpeedPrecision() {
    Serial.println("\n=== Low Speed Precision Test (below 1Hz) ===");
    
    pulseCounter.resetPosition();
    pwmStepper.enable();
    pwmStepper.setDirection(true);
    
    // Test very low frequencies
    float testFreqs[] = {0.1, 0.2, 0.5, 0.8, 1.0, 2.0, 5.0};
    int numFreqs = sizeof(testFreqs) / sizeof(testFreqs[0]);
    
    Serial.println("Testing low speed precision...");
    
    int passedLowSpeeds = 0;
    for (int i = 0; i < numFreqs; i++) {
        float freq = testFreqs[i];
        bool testPassed = false;
        float accuracy = 0.0;
        
        // For frequencies below 1Hz, we need special handling
        if (freq < 1.0) {
            Serial.print("Testing ultra-low speed: "); Serial.print(freq, 1); Serial.println(" Hz");
            
            // Calculate step period in milliseconds
            uint32_t stepPeriod = (uint32_t)(1000.0 / freq);
            int32_t startPos = pulseCounter.getPosition();
            
            Serial.print("  Step period: "); Serial.print(stepPeriod); Serial.println("ms");
            

            // Generate 10 steps manually with precise timing
            for (int step = 0; step < 10; step++) {
                uint32_t stepStart = millis();
                // Generate single pulse
                pwmStepper.startPWM(freq);
                // Wait for next step
                while (millis() - stepStart < stepPeriod) {
                    delay(1);
                }
                Serial.printf("    Step %d | Position: %d Interrupt cnt: %d\n", step + 1, pulseCounter.getPosition(), pwmStepper.getInterruptCount());
            }
            
            int32_t endPos = pulseCounter.getPosition();
            int32_t stepsCounted = endPos - startPos;
            accuracy = (float)stepsCounted / 10.0 * 100.0;
            testPassed = (stepsCounted >= 9 && stepsCounted <= 11); // ±1 step tolerance

            Serial.printf("  Expected: 10 steps, Counted: %d\n", stepsCounted);
            Serial.printf("  Accuracy: %.1f%%\n", accuracy);

            String freqName = "Ultra-low " + String(freq, 1) + "Hz";
            addTestResult(freqName, testPassed, 
                          "Expected: 10, Got: " + String(stepsCounted), accuracy);
            
        } else {
            // For frequencies >= 1Hz, use PWM
            Serial.print("Testing low speed PWM: "); Serial.print(freq, 1); Serial.println(" Hz");
            
            uint32_t freqHz = (uint32_t)freq;
            int32_t startPos = pulseCounter.getPosition();
            
            pwmStepper.startPWM(freqHz);
            delay(5000); // Run for 5 seconds
            pwmStepper.stopPWM();
            
            int32_t endPos = pulseCounter.getPosition();
            int32_t stepsCounted = endPos - startPos;
            uint32_t expectedSteps = freqHz * 5; // 5 seconds
            accuracy = (float)stepsCounted / expectedSteps * 100.0;
            testPassed = (accuracy >= 95.0); // Higher accuracy expected for PWM
            
            Serial.print("  Expected: "); Serial.print(expectedSteps);
            Serial.print(" steps, Counted: "); Serial.println(stepsCounted);
            Serial.print("  Accuracy: "); Serial.print(accuracy, 1); Serial.println("%");
            
            String freqName = "Low speed " + String(freq, 1) + "Hz";
            addTestResult(freqName, testPassed, 
                          "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                          accuracy);
        }
        
        if (testPassed) passedLowSpeeds++;
        delay(1000);
    }
    
    // Overall low speed test result
    bool overallLowSpeedOK = (passedLowSpeeds >= 5); // At least 5/7 speeds must pass
    addTestResult("Low Speed Overall", overallLowSpeedOK, 
                  String(passedLowSpeeds) + "/" + String(numFreqs) + " speeds passed");
    
    Serial.println("Low speed test completed!");
    Serial.print("Total position: "); Serial.println(pulseCounter.getPosition());
}

void runComprehensiveTests() {
    Serial.println("\n########################################");
    Serial.println("#     COMPREHENSIVE TEST SUITE        #");
    Serial.println("########################################");
    
    // Clear previous test results
    testCount = 0;
    
    testDirectionChanges();
    delay(2000);
    
    testCounterOverflow();
    delay(2000);
    
    testHighSpeedAcceleration();
    delay(2000);
    
    testLowSpeedPrecision();
    delay(2000);
    
    Serial.println("\n########################################");
    Serial.println("#     ALL TESTS COMPLETED             #");
    Serial.println("########################################");
    
    // Print comprehensive test summary
    printTestSummary();
}

void loop() {
    // Initialize test results for new cycle
    testCount = 0;
    
    Serial.println("\n=== MAIN MENU ===");
    Serial.println("1. Basic Demo (Position, Closed-loop, Speed)");
    Serial.println("2. Comprehensive Test Suite");
    Serial.println("3. Individual Tests");
    Serial.println("Starting Basic Demo in 3 seconds...");
    delay(3000);
    
    // Run basic demos first
    printSystemStatus();
    delay(1000);
    
    demonstratePositionTracking();
    delay(2000);
    
    demonstrateClosedLoopControl();
    delay(2000);
    
    demonstrateSpeedMeasurement();
    delay(2000);
    
    // Run comprehensive test suite
    runComprehensiveTests();
    
    pwmStepper.disable();
    pulseCounter.stop();
    
    Serial.println("\n=== ALL DEMOS AND TESTS COMPLETED ===");
    Serial.println("Restarting full cycle in 10 seconds...\n");
    delay(10000);
    
    pulseCounter.start();
}