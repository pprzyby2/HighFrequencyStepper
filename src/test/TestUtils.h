#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include <TMCStepper.h>
#include "ESP32Encoder.h"
#include "HighFrequencyStepper.h"

// Test result structure
struct TestResult {
    String stepperName;
    String testName;
    bool passed;
    String details;
    float accuracy;
};

// Global test results tracking (defined in TestUtils.cpp)
extern TestResult testResults[150];
extern int testCount;

// Test utility functions
void addTestResult(String stepperName, String testName, bool passed, String details = "", float accuracy = 0.0);
void printTestSummary();
void clearTestResults();
void printSystemStatus(HighFrequencyStepper& stepper);

#endif // TESTUTILS_H