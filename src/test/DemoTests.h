#ifndef DEMOTESTS_H
#define DEMOTESTS_H

#include <Arduino.h>
#include "PWMStepper.h"
#include "ESP32Encoder.h"
#include <TMCStepper.h>
#include "HighFrequencyStepper.h"

// Demonstration test functions
void demonstratePositionTracking(HighFrequencyStepper& stepper);
void demonstrateClosedLoopControl(HighFrequencyStepper& stepper);

#endif // DEMOTESTS_H