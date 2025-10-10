#ifndef SPEEDTESTS_H
#define SPEEDTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "ESP32Encoder.h"
#include "HighFrequencyStepper.h"

// Speed test functions
void testHighSpeedAcceleration(HighFrequencyStepper& controller);
void testLowSpeedPrecision(HighFrequencyStepper& controller);
void demonstrateSpeedMeasurement(HighFrequencyStepper& controller);

#endif // SPEEDTESTS_H