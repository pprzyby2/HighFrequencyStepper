#ifndef OVERFLOWTESTS_H
#define OVERFLOWTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "ESP32Encoder.h"
#include "HighFrequencyStepper.h"

// Overflow test functions
void testCounterOverflow(HighFrequencyStepper& controller);

#endif // OVERFLOWTESTS_H