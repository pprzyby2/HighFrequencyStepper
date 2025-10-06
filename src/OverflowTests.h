#ifndef OVERFLOWTESTS_H
#define OVERFLOWTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"

// Overflow test functions
void testCounterOverflow(PWMStepper& stepper, PulseCounter& counter);

#endif // OVERFLOWTESTS_H