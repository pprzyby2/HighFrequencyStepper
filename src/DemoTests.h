#ifndef DEMOTESTS_H
#define DEMOTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"
#include <TMCStepper.h>

// Demonstration test functions
void demonstratePositionTracking(PWMStepper& stepper, PulseCounter& counter);
void demonstrateClosedLoopControl(PWMStepper& stepper, PulseCounter& counter, TMC2209Stepper& driver);

#endif // DEMOTESTS_H