#ifndef MAXSPEEDTEST_H
#define MAXSPEEDTEST_H

#include "HighFrequencyStepper.h"

void testMaxSpeed(HighFrequencyStepper& controller);

void optimizeForMaxSpeed(HighFrequencyStepper& controller, uint8_t index);

#endif // MAXSPEEDTEST_H