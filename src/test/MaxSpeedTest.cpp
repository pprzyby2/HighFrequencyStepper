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
            int speed = 1000;

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
                speed += 1000; // Increase speed by 1kHz
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


