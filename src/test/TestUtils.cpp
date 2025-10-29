#include "TestUtils.h"

void addTestResult(String stepperName, String testName, bool passed, String details, float accuracy) {
    if (testCount < 150) {
        testResults[testCount].stepperName = stepperName;
        testResults[testCount].testName = testName;
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

    Serial.println("Stepper Name                | Test Name                    | Status | Details");
    Serial.println("--------------------------------------------------------------------|--------");
    
    for (int i = 0; i < testCount; i++) {
        String status = testResults[i].passed ? "PASS" : "FAIL";

        Serial.printf("%-29s | %-28s | %-6s | ", testResults[i].stepperName.c_str(), testResults[i].testName.c_str(), status.c_str());
        
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

void printSystemStatus(HighFrequencyStepper& stepper) {
    Serial.println("\n=== System Status ===");
    
    for (int i = 0; i < stepper.getStepperCount(); i++) {
        Serial.printf("\n--- Stepper %d Status ---\n", i);
        Serial.print("Enabled: "); Serial.println(stepper.isEnabled(i) ? "Yes" : "No");
        Serial.print("Position: "); Serial.println(stepper.getPosition(i));
        Serial.print("Target Position: "); Serial.println(stepper.getTargetPosition(i));
        Serial.print("Moving: "); Serial.println(stepper.isMoving(i) ? "Yes" : "No");
        Serial.print("Current Frequency: "); Serial.print(stepper.getCurrentFrequency(i)); Serial.println(" Hz");
        Serial.print("Microsteps: "); Serial.println(stepper.getMicrosteps(i));
        Serial.print("RMS Current: "); Serial.print(stepper.getRMSCurrent(i)); Serial.println(" mA");
        Serial.print("Max Frequency: "); Serial.print(stepper.getMaxFrequency(i)); Serial.println(" Hz");
        Serial.print("Acceleration: "); Serial.print(stepper.getAcceleration(i)); Serial.println(" steps/s²");
        Serial.print("Invert Direction: "); Serial.println(stepper.getInvertDirection(i) ? "Yes" : "No");
    }
    Serial.println("=====================\n");
}