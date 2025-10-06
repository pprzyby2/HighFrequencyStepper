#ifndef DIRECTIONTESTS_H
#define DIRECTIONTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "PulseCounter.h"

// Direction test functions
void testDirectionChanges(PWMStepper& stepper, PulseCounter& counter);

#endif // DIRECTIONTESTS_H