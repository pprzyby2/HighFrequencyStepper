#ifndef DIRECTIONTESTS_H
#define DIRECTIONTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "HighFrequencyStepper.h"

// Direction test functions
void testDirectionChanges(HighFrequencyStepper& stepper, int index);

#endif // DIRECTIONTESTS_H