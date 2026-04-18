#ifndef MANUALCONTROL_H
#define MANUALCONTROL_H

#include <Arduino.h>
#include "HighFrequencyStepper.h"
#include "LEDStatusIndicator.h"

// External references to main.cpp objects
extern HighFrequencyStepper stepperController;
extern LEDStatusIndicator ledStatus;

// Manual control state
extern bool manualControlMode;
extern uint8_t manualControlStepperIndex;
extern double manualControlSpeed;
extern bool manualControlDirection;

// Manual control functions
void printManualControlMenu();
void processManualControlInput();

// Individual control functions
void manualControlEnableStepper();
void manualControlDisableStepper();
void manualControlStart();
void manualControlStop();
void manualControlReverseDirection();
void manualControlIncreaseSpeed(double increment);
void manualControlDecreaseSpeed(double decrement);
void manualControlSelectNextStepper();
void manualControlShowStatus();
void manualControlZeroPosition();

// TMC2209 specific controls
void manualControlIncreaseRMSCurrent();
void manualControlDecreaseRMSCurrent();
void manualControlToggleStealthChop();
void manualControlSetStallGuardThreshold();
void manualControlChangeMicrosteps();
void manualControlShowTMCInfo();

#endif // MANUALCONTROL_H
