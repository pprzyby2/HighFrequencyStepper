#ifndef STATETRANSITIONTESTS_H
#define STATETRANSITIONTESTS_H

#include <Arduino.h>
#include "HighFrequencyStepper.h"

/**
 * @brief Validate PWMStepper state transitions between position, frequency and stop modes.
 *
 * Covered transitions:
 * - STEPPER_MOVE_TO_POSITION -> STEPPER_MOVE_WITH_FREQUENCY
 * - STEPPER_MOVE_WITH_FREQUENCY -> STEPPER_MOVE_TO_POSITION
 * - transitions from both moving modes to STOP
 * - transitions from STOP back to both moving modes
 */
void testStateTransitions(HighFrequencyStepper& stepper);

#endif // STATETRANSITIONTESTS_H
