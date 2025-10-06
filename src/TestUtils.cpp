#include "TestUtils.h"

void addTestResult(String name, bool passed, String details, float accuracy) {
    if (testCount < 30) {
        testResults[testCount].testName = name;
        testResults[testCount].passed = passed;
        testResults[testCount].details = details;
        testResults[testCount].accuracy = accuracy;
        testCount++;
    }
}

void clearTestResults() {
    testCount = 0;
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

void printSystemStatus(PWMStepper& stepper, PulseCounter& counter, TMC2209Stepper& driver) {
    Serial.println("\n=== System Status ===");
    
    // PWM Stepper status
    Serial.println("PWM Stepper:");
    Serial.print("  Enabled: "); Serial.println(stepper.isEnabled() ? "Yes" : "No");
    Serial.print("  Direction: "); Serial.println(stepper.getDirection() ? "Forward" : "Reverse");
    Serial.print("  Frequency: "); Serial.print(stepper.getFrequency()); Serial.println(" Hz");
    Serial.print("  Mode: "); Serial.println(stepper.isInLEDCMode() ? "LEDC" : "Timer");
    
    // Pulse Counter status
    Serial.println("Pulse Counter:");
    counter.printStatus();
    
    // TMC2209 status
    Serial.println("TMC2209:");
    Serial.print("  Connection: "); Serial.println(driver.test_connection());
    Serial.print("  SG Result: "); Serial.println(driver.SG_RESULT());
    Serial.print("  Current: "); Serial.print(driver.cs2rms(driver.cs_actual())); Serial.println(" mA");
}