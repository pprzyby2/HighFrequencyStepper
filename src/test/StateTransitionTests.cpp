#include "StateTransitionTests.h"
#include "TestUtils.h"

namespace {

bool waitForMoveToPosition(HighFrequencyStepper& stepper, uint8_t index, uint32_t timeoutMs) {
    delay(2000);
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (stepper.isMoving(index) && stepper.isMovingToPosition(index)) {
            return true;
        }
        delay(5);
    }
    return false;
}

bool waitForMoveWithFrequency(HighFrequencyStepper& stepper, uint8_t index, uint32_t timeoutMs) {
    delay(2000);
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (stepper.isMoving(index) && !stepper.isMovingToPosition(index)) {
            return true;
        }
        delay(5);
    }
    return false;
}

bool waitForStopped(HighFrequencyStepper& stepper, uint8_t index, uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (!stepper.isMoving(index) && abs(stepper.getCurrentFrequency(index)) < 1.0) {
            return true;
        }
        delay(5);
    }
    return false;
}

void addTransitionResult(
    const String& stepperName,
    const String& testName,
    bool passed,
    HighFrequencyStepper& stepper,
    uint8_t index)
{
    String details = String("moving=") + (stepper.isMoving(index) ? "YES" : "NO") +
                     ", movingToPosition=" + (stepper.isMovingToPosition(index) ? "YES" : "NO") +
                     ", freq=" + String(stepper.getCurrentFrequency(index), 2);
    addTestResult(stepperName, testName, passed, details);
}

}

void testStateTransitions(HighFrequencyStepper& stepper) {
    Serial.println("\n=== State Transition Test ===");
    Serial.println("Validating mode transitions for each configured stepper...\n");

    for (uint8_t index = 0; index < stepper.getStepperCount(); index++) {
        String stepperName = stepper.getName(index);
        Serial.printf("--- Testing %s (index %d) ---\n", stepperName.c_str(), index);

        stepper.enableStepper(index);
        stepper.stop(index);
        stepper.setPosition(index, 0);
        delay(50);

        int32_t stepsPerRev = stepper.getMicrostepsPerRevolution(index);
        double maxFreq = stepper.getMaxFrequency(index);
        double freqA = max(20.0, maxFreq * 0.20);
        double freqB = max(20.0, maxFreq);
        int32_t targetA = max(stepsPerRev * 10, 200);

        // STOP -> MOVE_TO_POSITION
        stepper.moveToPosition(index, targetA, freqA, false);
        bool stopToMoveToPos = waitForMoveToPosition(stepper, index, 1000);
        addTransitionResult(stepperName, "STOP -> MOVE_TO_POSITION", stopToMoveToPos, stepper, index);

        // MOVE_TO_POSITION -> MOVE_WITH_FREQUENCY
        stepper.moveAtFrequency(index, freqA);
        bool moveToPosToFreq = waitForMoveWithFrequency(stepper, index, 1000);
        addTransitionResult(stepperName, "MOVE_TO_POSITION -> MOVE_WITH_FREQUENCY", moveToPosToFreq, stepper, index);

        // MOVE_WITH_FREQUENCY -> STOP
        stepper.stop(index);
        bool moveFreqToStop = waitForStopped(stepper, index, 1500);
        addTransitionResult(stepperName, "MOVE_WITH_FREQUENCY -> STOP", moveFreqToStop, stepper, index);

        // STOP -> MOVE_WITH_FREQUENCY
        stepper.moveAtFrequency(index, -freqB);
        bool stopToMoveFreq = waitForMoveWithFrequency(stepper, index, 1000);
        addTransitionResult(stepperName, "STOP -> MOVE_WITH_FREQUENCY", stopToMoveFreq, stepper, index);

        // MOVE_WITH_FREQUENCY -> MOVE_TO_POSITION
        int32_t currentPos = stepper.getPosition(index);
        int32_t targetB = currentPos + max(stepsPerRev / 8, 200);
        stepper.moveToPosition(index, targetB, freqB, false);
        bool moveFreqToMovePos = waitForMoveToPosition(stepper, index, 1000);
        addTransitionResult(stepperName, "MOVE_WITH_FREQUENCY -> MOVE_TO_POSITION", moveFreqToMovePos, stepper, index);

        // MOVE_TO_POSITION -> STOP
        stepper.stop(index);
        bool movePosToStop = waitForStopped(stepper, index, 1500);
        addTransitionResult(stepperName, "MOVE_TO_POSITION -> STOP", movePosToStop, stepper, index);

        Serial.printf("Completed transition checks for %s\n\n", stepperName.c_str());
    }

    cleanupAfterTest(stepper);
    Serial.println("=== State Transition Test Complete ===\n");
}
