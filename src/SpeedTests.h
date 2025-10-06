#ifndef SPEEDTESTS_H
#define SPEEDTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"

// Speed test functions
void testHighSpeedAcceleration(PWMStepper& stepper, PulseCounter& counter);
void testLowSpeedPrecision(PWMStepper& stepper, PulseCounter& counter);
void demonstrateSpeedMeasurement(PWMStepper& stepper, PulseCounter& counter);

#endif // SPEEDTESTS_H