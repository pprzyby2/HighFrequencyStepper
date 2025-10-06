#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"
#include <TMCStepper.h>

// Test result structure
struct TestResult {
    String testName;
    bool passed;
    String details;
    float accuracy;
};

// Global test results tracking
static TestResult testResults[30]; // Increased capacity
static int testCount = 0;

// Test utility functions
void addTestResult(String name, bool passed, String details = "", float accuracy = 0.0);
void printTestSummary();
void clearTestResults();
void printSystemStatus(PWMStepper& stepper, PulseCounter& counter, TMC2209Stepper& driver);

#endif // TESTUTILS_H