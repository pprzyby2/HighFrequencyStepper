#include "MaxSpeedTest.h"
#include "TestUtils.h"

void testMaxSpeed(HighFrequencyStepper& controller) {
    Serial.println("\n=== Maximum Speed Test ===");
    Serial.println("Testing maximum achievable speed for each stepper...");

    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        if (!controller.isValidIndex(i)) {
            Serial.printf("Stepper %d not configured properly. Skipping...\n", i);
            continue;
        }

        controller.setPosition(i, 0);
        controller.enableStepper(i);
        int microstepsPerRev = controller.getMicrostepsPerRevolution(i);

        for (int dir = 0; dir < 2; dir++) {

            Serial.printf("\nTesting Stepper %d (%s)...\n", i, dir == 1 ? "Forward" : "Backward");

            bool positionOk = true;
            int microsteps = controller.getMicrosteps(i);
            int speed = 1000 * microsteps / 16;
            while (positionOk && speed <= controller.getMaxFrequency(i)) {
                Serial.printf("  Testing speed: %d Hz\n", speed);

                controller.accelerateToFrequency(i, speed, dir, true);
                uint32_t startTime = millis();
                int32_t startPos = controller.getPosition(i);
                delay(1000); // Run for 1 second at each speed
                uint32_t endTime = millis();
                int32_t endPos = controller.getPosition(i);

                uint32_t duration = endTime - startTime;
                int32_t stepsCounted = endPos - startPos;
                if (dir == 0) stepsCounted = -stepsCounted; // Adjust for backward direction
                uint32_t expectedSteps = (speed * duration) / 1000;
                float accuracy = (float)stepsCounted / expectedSteps * 100.0;

                Serial.printf("    Duration: %d ms | Expected steps: %d | Counted: %d | RPM: %.1f | Accuracy: %.1f%%\n",
                            duration, expectedSteps, stepsCounted, (float) (60.0 * stepsCounted) / microstepsPerRev * (duration / 1000.0), accuracy);

                bool speedTestOK = (accuracy >= 90.0);
                String speedName = "Stepper " + String(i) + " Speed " + String(speed) + "Hz";
                addTestResult(controller.getName(i), speedName, speedTestOK,
                            "Expected: " + String(expectedSteps) + ", Got: " + String(stepsCounted), 
                            accuracy);
                if (!speedTestOK) {
                    Serial.println("    WARNING: Low accuracy detected! Stopping further speed tests.");
                    positionOk = false; // Stop testing higher speeds if accuracy drops
                }
                speed += (1024 * microsteps / 16); // Increase speed by 1kHz
                delay(200); // Brief pause between speeds

            }
            controller.accelerateToFrequency(i, 0, dir, true); // Gently stop the stepper
            delay(1000);
        }
        controller.stop(i); // Stop the stepper
        Serial.printf("Finished testing Stepper %d.\n", i);
        controller.disableStepper(i);
    }
    Serial.println("Maximum speed test completed!");
    for (uint8_t i = 0; i < controller.getStepperCount(); i++) {
        Serial.printf("Final position of Stepper %d: %d\n", i, controller.getPosition(i));
    }
    Serial.println("All steppers disabled.");
    controller.disableAll();
    delay(1000);
    Serial.println("Test sequence finished.");
}

int32_t getMaxAchievableSpeed(HighFrequencyStepper& controller, uint8_t index) {
    controller.setPosition(index, 0);
    controller.enableStepper(index);
    int microsteps = controller.getMicrosteps(index);
    int deltaSpeed = 1000 * microsteps / 16;

    int32_t maxValidSpeed = 0;
    int testTime = 100;
    for (int32_t testSpeed = deltaSpeed; testSpeed <= controller.getMaxFrequency(index); testSpeed += deltaSpeed) {
        controller.accelerateToFrequency(index, testSpeed, true, true);
        controller.setPosition(index, 0);
        delay(testTime); // Run for 100 ms
        int32_t endPos = abs(controller.getPosition(index));
        int32_t expectedPos = (int) float(testSpeed * testTime) / 1000.0;
        //Serial.printf("%d Hz -> %d Err \n", testSpeed, endPos - expectedPos);
        if (abs(endPos - expectedPos) > (expectedPos * 0.1)) { // If position is not close to expected
            break;
        }
        maxValidSpeed = testSpeed;
    }
    return maxValidSpeed;
}

void optimizeForMaxSpeed(HighFrequencyStepper& controller, uint8_t index) {
    Serial.printf("\nOptimizing maximum speed for Stepper %d...\n", index);
    Serial.printf("Searching for optimal StallGuard threshold setting...\n");
    int microsteps = controller.getMicrosteps(index);
    int deltaSpeed = 1000 * microsteps / 16;

    uint16_t bestSGTHRS = 0;
    int32_t globalMaxSpeed = 0;
    for (uint16_t sgthrs = 0; sgthrs <= 255; sgthrs += 8) {
        controller.setStallGuardThreshold(index, sgthrs);
        Serial.printf("  Testing SGTHRS: %d\n", sgthrs);

        controller.setPosition(index, 0);
        controller.enableStepper(index);
        int32_t maxValidSpeed = getMaxAchievableSpeed(controller, index);
        if (maxValidSpeed > globalMaxSpeed) {
            globalMaxSpeed = maxValidSpeed;
            bestSGTHRS = sgthrs;
        }
        Serial.printf("    Max valid speed at SGTHRS %d: %d Hz %.2f RPM\n", sgthrs, maxValidSpeed, 
            controller.getMicrostepsPerRevolution(index) > 0 ? (float)(60.0 * maxValidSpeed) / controller.getMicrostepsPerRevolution(index) : 0.0);
        delay(500);
    }
    Serial.printf("    Optimal SGTHRS for Stepper %d: %d achieving max speed of %d Hz\n", index, bestSGTHRS, globalMaxSpeed);

    uint16_t bestRMS = 0;
    globalMaxSpeed = 0;
    for (uint16_t rms = 100; rms < 1600; rms += 50) {
        controller.setRMSCurrent(index, rms);
        Serial.printf("  Testing RMS: %d\n", rms);

        controller.setPosition(index, 0);
        controller.enableStepper(index);
        int testSpeed = controller.getMaxFrequency(index);
        int32_t maxValidSpeed = getMaxAchievableSpeed(controller, index);
        if (maxValidSpeed > globalMaxSpeed) {
            globalMaxSpeed = maxValidSpeed;
            bestRMS = rms;
        }
        Serial.printf("    Max valid speed at RMS %d: %d Hz %.2f RPM\n", rms, maxValidSpeed, 
            controller.getMicrostepsPerRevolution(index) > 0 ? (float)(60.0 * maxValidSpeed) / controller.getMicrostepsPerRevolution(index) : 0.0);
        delay(500);
    }
    Serial.printf("Optimal RMS for Stepper %d: %d achieving max speed of %d Hz\n", index, bestRMS, globalMaxSpeed);    
}
